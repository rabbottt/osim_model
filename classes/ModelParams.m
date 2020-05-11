classdef ModelParams
    
    properties (Constant)
        model_folder = 'C:\Users\16179\Documents\GitHub\utilities'
        geo_folder = 'C:\OpenSim 4.0\Geometry'
    end
    
    properties 
        model_name      % string, name of model before .osim
        J               % Frame Jacobian                
        R               % Moment Arm Matrix for All Muscles
        Fa              % Strength for All Muscles
        nMuscles        % Total number of individual muscles 
        nGroups         % Number of muscle groups
        nDOFs           % Number of independent coordinates
        group_idx       % Group index by muscle 
        mm_groups       % cell array of muscles in each group
        Sm              % (nGroups x nMuscles) binary matrix to group muscles
        
        osim            % structure of osim classes constructed in constructor
                        %   .model, 
        
        joint_names     % njoints x 1 cell array of joint names strings
        coord_names     % nDOFs x 1 cell array of coordinate name strings 
        muscle_names    % nMuscles x 1 cell array of muscle name strings
    end
    
    methods
        
        %constructor
        function obj = ModelParams(model_string)  
            
            obj.model_name = model_string;
            
            % osim libraries
            import org.opensim.modeling.*;
            % path to Geometry folder to avoid error messages          
            ModelVisualizer.addDirToGeometrySearchPaths(obj.geo_folder);
            
            % initialize OpenSim model object
            model_path = fullfile(obj.model_folder,[obj.model_name '.osim']);
            obj.osim.model = Model(model_path);
            obj.osim.model.initSystem();
            
            % get sets
            obj.osim.jointset = obj.osim.model.getJointSet();
            obj.osim.coordset = obj.osim.model.getCoordinateSet();
            obj.osim.muscleset = obj.osim.model.getMuscles();
            
            % get dimensions
            obj.nDOFs = obj.osim.coordset.getSize(); 
            obj.nMuscles = obj.osim.muscleset.getSize();             
            
            % get names from model
            obj.joint_names = obj.getJointNames();
            obj.coord_names = obj.getCoordNames();
            obj.muscle_names = obj.getMuscleNames();                       
                       
            % start with neutral posture            
            posture_scalar = 0;
            posture = posture_scalar*ones(1,obj.nDOFs);            
            
            % Initialize state of model
            obj.osim.state = obj.osim.model.initializeState(); 
            obj.updateModelPosture(posture); 
            
            % get max isometric strength from osim model
            obj.Fa = obj.getMuscleStrengths();
            
            % get Jacobian J and Moment Arm Matrix R for model configuration
            obj.R = obj.calcMomentArms(obj.osim.state);
            obj.J = obj.getFrameJacobian(obj.osim.state);

        end
        
        function obj = updateMuscleGroups(obj, muscle_groups)
            % check dim of input   
            nM = obj.nMuscles;
            if ~isequal(length(muscle_groups),nM)
                error('muscle_groups input must be length of nMuscles')
            end
            
            obj.group_idx = muscle_groups;
            obj.nGroups = max(muscle_groups);
            obj.Sm = obj.computeSm(muscle_groups); 
            obj.mm_groups = obj.muscleNamesInGroups();
            
        end
        
        function names = getJointNames(obj)
            jointset = obj.osim.jointset;
            nJ = jointset.getSize();
            names = {};
            for ii = 1 : nJ
                names{ii} = char(jointset.get(ii-1).toString);
            end
            names = names';
            
        end
        
        function names = getCoordNames(obj)
            
            coordset = obj.osim.coordset;
            nC = obj.nDOFs;
            names = {};
            for ii = 1 : nC
                names{ii} = char(coordset.get(ii-1).toString);
            end
            names = names';
            
        end
        
        function names = getMuscleNames(obj)
            
            muscleset = obj.osim.muscleset;
            nM = obj.nMuscles;
            names = {};
            for mm = 1 : nM
                names{mm} = char(muscleset.get(mm-1).toString);
            end
            names = names';
            
        end
        
        function updateModelPosture(obj, posture)            
            
            % check that coord_names and nDOFs have been set 
            if isempty(obj.coord_names) || isempty(obj.nDOFs)
                error('obj.coord_names or obj.nDOFs is empty')
            end
            % set that posture is the correct length 
            if ~isequal(obj.nDOFs,length(posture))
                error('Length of Posture must equal the nDOFs')
            end            
                       
            % update coordinates
            coordset = obj.osim.model.updCoordinateSet();
            for jj = 1 : obj.nDOFs
                coordinate = coordset.get(obj.coord_names{jj});
                coordinate.setValue(obj.osim.state, deg2rad(posture(jj))); 
            end
            
            obj.osim.model.realizePosition(obj.osim.state);
            obj.osim.model.equilibrateMuscles(obj.osim.state);            
            
            % should recompute Jacobian
        end
        
        
        function Fmax = getMuscleStrengths(obj)
            % Gets state independent muscle parameters from OpenSim Model 
            % Only for muscles included in obj.muscle_names class property
            % Fa:  Muscle.getMaxIsometricForce()
            
            muscleset = obj.osim.model.getMuscles();
            nM = muscleset.getSize();
            Fmax = zeros(nM,1);
            for mm = 1 : nM
                muscle = muscleset.get(mm-1);
                Fmax(mm) = muscle.getMaxIsometricForce();
            end
        end
        
        function R = calcMomentArms(obj, state) 
            
            % get component osim objects 
            muscleset = obj.osim.model.getMuscles();
            nM = muscleset.getSize();
            coordset = obj.osim.model.getCoordinateSet();
            nC = coordset.getSize();
            
            R = zeros(nC, nM);
            for mm = 1 : nM
                muscle = muscleset.get(mm-1);
                for jj = 1 : obj.nDOFs 
                    coord = coordset.get(jj-1);
                    % Get moment arm for this muscle for this coordinate
                    R(jj,mm) = muscle.computeMomentArm(state, coord);
                end
            end
            % Set very small numbers to 0
            R(abs(R)<1e-10) = 0;
        end

        function qV = getQVector(obj, state)

            %get current values of each generalized coordinate from the current state
            coordset = obj.osim.model.getCoordinateSet();
            nC = coordset.getSize();
            qV = zeros(nC,1);
            for qq = 1 : nC
                qV(qq) = coordset.get(qq-1).getValue(state);
            end   
            qV(abs(qV) < 1e-12) = 0;

        end
        
        function J = getFrameJacobian(obj, state)
            %
            import org.opensim.modeling.*;
            
            osimJ = Matrix();
            body_index = obj.osim.model.getBodySet().get('skull').getMobilizedBodyIndex();
            station_vec = Vec3(0,0,0); % stay at frame of skull
            smss = obj.osim.model.getMatterSubsystem();

            smss.calcFrameJacobian(state, body_index, station_vec, osimJ);
            J = obj.osimMatrixToArray(osimJ);
            J(abs(J) < 1e-12) = 0;
        
        end        
               
        function Sm = computeSm(obj, muscle_groups)

            nM = length(muscle_groups);
            nG = obj.nGroups;
            
            Sm = zeros(nM, nG);
            for mm = 1 : nM
               idx = muscle_groups(mm);
               if idx ~= 0
                   Sm(mm,idx) = 1;
               end    
            end 
        end
        
        function grps = muscleNamesInGroups(obj)

            nG = obj.nGroups;           
            
            grps = cell(nG,1);
            for gg = 1 : nG
                grps{gg} = obj.muscle_names(obj.group_idx == gg);
            end           
            
        end
        
    end
        
    methods (Static)

        function a = osimMatrixToArray(p)
            % import Java Libraries
            import org.opensim.modeling.*

             % Check the input type, will work for any of these
            if strcmp(class(p), 'org.opensim.modeling.Matrix')

                 % Convert the input Opensim Matrix to a Matlab Array
                 pdim = [nrow(p),ncol(p)];

                 a = zeros(pdim);
                 for rr = 1 : pdim(1)
                     for cc = 1 : pdim(2)
                         a(rr,cc) = p.get(rr-1,cc-1);
                     end
                 end
            else
                error('Input must be org.opensim.modeling.Matrix! 0_o ') 
            end
        end

    end        
        
end