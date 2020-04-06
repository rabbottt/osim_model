%  ===== osim Class Functions =====
%
%
% % MTL = calcMTL(obj, state, muscleList)
% % MusPara = getMuscleConstants(obj, muscleList) 
% % [Lfn, cosa] = calcFiberLengths(MTL, TSL, L0, a0)
% % Factive = FaScalingFactor(Fmax, afl, Lfn, cosa)
% % Fpassive = FpScalingFactor(Fmax, pfl, Lfn)
% % R = calcMomentArms(obj, state) 
% % Minv = getMinv(obj, state)
% % [] = updateCoordinates(obj, state, jointAngles)
% % qVector = getQVector(obj, state)
% % J = getJacobian(obj, state)
% % Fbushing = calcBushingForces(obj, state, bushingList)
% % axes = getBodyTransforms(obj, state)
% % [laserIntersect, check] = laserWallLocation(obj, state)
% 
% 
% % % From utility.m class these methods are used: 
% %         rad = deg2rad(deg)
% %         array = osimMatrixToArray(p)
% %         vec = osimVec3FromArray(p)
% %         vec = osimVec3ToArray(p)
% %         array = osimVectorToArray(p)
% %
% %
% 
% %
% % ==================================

classdef osimClass
    
    properties (Constant)
        
        %StationVector = [0.1, 0.1, 0.0]; %temp debug
        %laserVector = [ 10.0, 0.1, 0.0 ]; %vector in skull frame. Laser travels at least 10m
        distance_from_wall = 2.0; %wall is 2 meters away        
        zero_cut = 1e-12; % set values less than this value to 0
    end

   
    properties % defined with call to constructor obj = osim(model_path)
        model
        laserMarker
        osimMatrix
        osimVec3
        osimVec6
        skullframe
        skullbody
        spinebody
        forceset
        muscleset
        coordset
        bodyset
        head_mass
        
        %not initialized in constructor
        %muscleList = ModelConstants.MuscleList_reduced; %default
        muscleList = ModelConstants.MuscleList;
        IndCoordList = ModelConstants.CoordList6; %default 6 independent coords
        CoordList = ModelConstants.CoordList24; %default 24 coords (some coupled)
        bushingList = ModelConstants.BushingForceList; %default
        StationVector = [ 0.1, 0.1, 0.0 ]; %default: forehead
        laserVector = [ 10.0, 0.1, 0.0 ]; %default: 10m from forehead
    end

    
    methods
        
        function obj = osimClass(model_path)           
            import org.opensim.modeling.*;
            obj.model = Model(model_path);
            obj.model.initSystem();
            obj.osimMatrix = Matrix();            
            obj.osimVec3 = Vec3();
            obj.osimVec6 = Vec6();
            obj.skullframe = obj.model.getBodySet().get('skull').getMobilizedBodyIndex();
            obj.skullbody = obj.model.getBodySet().get('skull');
            obj.spinebody = obj.model.getBodySet().get('spine');
            try
                obj.laserMarker = obj.model.getMarkerSet().get('Laser');
            catch
                disp('No marker called Laser')
            end
            obj.forceset = obj.model.getForceSet();
            obj.muscleset = obj.model.getMuscles();
            obj.coordset = obj.model.getCoordinateSet();
            obj.bodyset = obj.model.getBodySet();
            obj.head_mass = obj.model.getBodySet().get('skull').getMass(); %original head mass
        end
        
        function obj = setMuscleList(obj, muscle_list)
            obj.muscleList = muscle_list;
        end
        
        function obj = setIndCoordList(obj, ind_coord_list)
            obj.IndCoordList = ind_coord_list;
        end
        
        function obj = setCoordList(obj, coord_list)
            obj.CoordList = coord_list;
        end
        
        function obj = setBushingList(obj, bushing_list)
            obj.bushingList = bushing_list;
        end
        
        function obj = setStationVector(obj, station_vec)
            obj.StationVector = station_vec;
        end
        
        %spine_orientation options: 'upright', 'prone', 'supine',
        %'fwd_lean', 'back_lean' . The default is 'upright'
        function changeBodyOrientation(obj, spine_orientation)
            import org.opensim.modeling.*;
            
            switch spine_orientation
                case 'upright'
                    spine_angle = Vec3(0, 0, 0);
                case 'prone'
                    pitch_angle = deg2rad(-90);
                    spine_angle = Vec3(0, 0, pitch_angle);
                case 'supine'
                    pitch_angle = deg2rad(90);
                    spine_angle = Vec3(0, 0, pitch_angle);
                case 'fwd_lean'
                    pitch_angle = deg2rad(-45);
                    spine_angle = Vec3(0, 0, pitch_angle);
                case 'back_lean'
                    pitch_angle = deg2rad(45);
                    spine_angle = Vec3(0, 0, pitch_angle);
                otherwise
                    spine_angle = Vec3(0, 0, 0); %default to upright
            end
            %get spine joint (indexed at 0 in model)
            spine_joint = obj.model.getJointSet.get(0);
            %get handle to the 'spine_offset' PhysicalOffsetFrame
            spine_offset_frame = spine_joint.get_frames(0); 
            % set frame 0 to the orientation in radians selected above
            spine_offset_frame.set_orientation(0,spine_angle);
            
            %must reinitialize/rebuild system after changing PhysicalOffsetFrame
            obj.model.initSystem(); 
            
        end
        
        % set whether coordinate coupler constraints are enforced (if set
        % to 0, will need to supply full set of 24 coordinates instead of
        % just the 6 independent coordinates)
        % note: indices start with 0, not one, from OpenSim
        function setEnforceConstraints(obj,enforce,indices)
                        
            import org.opensim.modeling.*;
            
            constraintSet = obj.model.getConstraintSet;
            nConstraints = obj.model.getNumConstraints;
            
            switch nargin
                case 2 %default to set all constraints to boolean enforce
                    for cc = 1 : nConstraints
                        constraintSet.get(cc-1).set_isEnforced(enforce)
                    end
                case 3 %set only the constraints specified with indices input
                    for cc = 1 : length(indices)
                        ii = indices(cc);
                        constraintSet.get(ii).set_isEnforced(enforce);
                    end
                otherwise
                    disp('Wrong number of inputs for setEnforceConstraints, check inputs.');
            end
            
            
        end
        
        function mm_string = getMuscleNames(obj)
            % gets all muscle names from OpenSim Model
            % (not from osim class muscleList property)
            
            import org.opensim.modeling.*;

            muscles = obj.model.getMuscles();
            nMuscles = muscles.getSize();
            for i = 1 : nMuscles
                mm(i) = string(muscles.get(i-1).getName());
            end
            mm_string = mm';
        end
        
        function MusPara = getMuscleConstants(obj)
            % Gets state independent muscle parameters from OpenSim Model 
            % Only for muscles included in obj.muscleList class property
            % MusPara struct includes:  Fmax:  Muscle.getMaxIsometricForce()
            %                           L0: Muscle.getOptimalFiberLength()
            %                           TSL: Muscle.getTendonSlackLength()
            %                           a0: Muscle.getPennationAngleAtOptimalFiberLength
            
            muscle_list = obj.muscleList;
            nMuscles = length(muscle_list);
        
            for mm = 1 : nMuscles
                muscle = obj.muscleset.get(muscle_list{mm});
                MusPara.Name(mm,1) = string(muscle.getName());
                MusPara.Fmax(mm,1) = muscle.getMaxIsometricForce();
                MusPara.L0(mm,1) = muscle.getOptimalFiberLength();
                MusPara.TSL(mm,1) = muscle.getTendonSlackLength();
                MusPara.a0(mm,1) = muscle.getPennationAngleAtOptimalFiberLength();
            end
        end
        
        % % % 
        % 
        % Scale the max iso force for the loaded model (does not save to
        % model)
        % INPUTS:   mm_indices: array of muscle indices to scale
        %           scaleFactor: factor to scale muscles by 
        %           mmConstants: Structure used to store muscle parameters
        %                   must contain mmConstants.Fmax field
        %           
        %           
        % OUTPUTS:  mmConstants: updated muscle parameters
        % 
        %  hint: use mm_indices = find(Sm(:,mm_group))
        % % % 
        
        function mmConstants = scaleMaxIsoForce(obj,mm_indices, scaleFactor, mmConstants)
            import org.opensim.modeling.*;
            muscles = obj.model.getMuscles();

            for ii = 1 : length(mm_indices)
                mm_i = mm_indices(ii);
                muscle = muscles.get(mm_i-1);
                oldMaxForce = muscle.getMaxIsometricForce();
                newMaxForce = oldMaxForce * scaleFactor;
                muscle.setMaxIsometricForce(newMaxForce);
                
                fprintf('max force of %s changed from %.2f to %.2f\n',mmConstants.Name(mm_i),oldMaxForce,newMaxForce);
                mmConstants.Fmax(mm_i,1) = muscle.getMaxIsometricForce();
            end
        end

        function MTL = calcMTL(obj, state)
            import org.opensim.modeling.*;
            
            muscle_list = obj.muscleList;
            nMuscles = length(muscle_list);
            MTL = zeros(nMuscles,1);
            
            for mm = 1 : nMuscles
                muscle = obj.muscleset.get(muscle_list{mm});    
                % Get state dependent musculotendon length
                MTL(mm,1) = muscle.getLength(state);
            end
        end
        
        
        
        function R = calcMomentArms(obj, state) 
            
            muscle_list = obj.muscleList;
            nMuscles = length(muscle_list);
            coord_list = obj.CoordList; %all coordinates (not just independent)
            nCoords = length(coord_list);

            R = zeros(nCoords, nMuscles);

            for mm = 1 : nMuscles
                muscle = obj.muscleset.get(muscle_list{mm});
                for jj = 1 : nCoords 
                    joint = obj.model.updCoordinateSet().get(coord_list{jj}); %all coords
                    % Get moment arms for given posture
                    Rjm = muscle.computeMomentArm(state, joint); % temporarily assigned as R
                    Rjm(abs(Rjm)<obj.zero_cut) = 0;
                    R(jj,mm) = Rjm;
                end
            end
        end
        
        function Minv = getMinv(obj, state)
    
            % create output variable of OpenSim Matrix class
            osimMinv = obj.osimMatrix;

            % call to OpenSim API returns last argument (osimMinv)
            smss = obj.model.getMatterSubsystem();
            smss.calcMInv(state, osimMinv); 

            % convert from OpenSim object to Matlab array
            Minv = utility.osimMatrixToArray(osimMinv);
            % CHECK THIS
            Minv(abs(Minv)<obj.zero_cut)=0;

        end
        
        function [] = updateCoordinates(obj, state, posture)
            nAngles = length(posture);            
            nCoords = length(obj.IndCoordList);
            
            if nAngles ~= nCoords
                error('List of coordinate names must be same length as number of joint angles.  Check your inputs! 0_o');
            end            
            
            for jj = 1 : nCoords
                coordinate = obj.model.updCoordinateSet().get(obj.IndCoordList{jj});
                % DoFs should are set in radians
                % enforce constraints only after all coords have been set
                coordinate.setValue(state, utility.deg2rad(posture(jj)), double(jj==nCoords)); 
            end

        end
        
         
        function qVector = getQVector(obj, state)

            %get updated Qs
            nCoords = obj.model.getNumCoordinates();
            qVector = zeros(1, nCoords);

            %get current values of each generalized coordinate from the current state
            for qq = 1 : nCoords
                qVi = obj.coordset.get(qq-1).getValue(state);
                qVi(abs(qVi) < obj.zero_cut) = 0;
                qVector(qq) = qVi;
            end   

        end
        
        function J = getJacobian(obj, state)
    
            osimJ = obj.osimMatrix;
            %body frame
            body_index = obj.skullframe;
            
            %location of point in body frame
            station_vector = utility.osimVec3FromArray(obj.StationVector); 
            
            %debug
            smss = obj.model.getMatterSubsystem();

            smss.calcStationJacobian(state, body_index, station_vector, osimJ);
            J = utility.osimMatrixToArray(osimJ);
            J(abs(J) < obj.zero_cut) = 0;

        end
        
        function J = getFrameJacobian(obj, state)
            osimJ = obj.osimMatrix;
            body_index = obj.skullframe;
            fh_vec = utility.osimVec3FromArray(obj.StationVector);
            smss = obj.model.getMatterSubsystem();

            smss.calcFrameJacobian(state, body_index, fh_vec, osimJ);
            J = utility.osimMatrixToArray(osimJ);
            J(abs(J) < obj.zero_cut) = 0;
        end
        
        function J = getSysJacobian(obj, state)
            osimJ = obj.osimMatrix;
            smss = obj.model.getMatterSubsystem();

            smss.calcSystemJacobian(state, osimJ);
            J = utility.osimMatrixToArray(osimJ);
            J(abs(J) < obj.zero_cut) = 0;
        end
        
        function Fgrav = getGravityForces(obj, state)
            % import OpenSim Libraries
            import org.opensim.modeling.*; 
            
            % get references to OpenSim Objects
            bodies = obj.bodyset;
            grav = obj.model.getGravity();
            ground = obj.model.getGround();
            
            % number of bodies and coordinates
            nbods = obj.model.getNumBodies();
            ncoords = obj.model.getNumCoordinates();

            %start GravF vector with zeros for ground mobodIndex = 0;
            GravF_G = zeros(6,1); 
    
            % iterate through each body; get gravity vector in ground frame
            for bb = 1 : nbods
                
                body = bodies.get(bb-1);
                mass = body.getMass();
                CoM = utility.osimVec3ToArray(body.getMassCenter())';
                
                % Rotation Matrix from body to ground frame
                R_GB = osimMatrixToArray(body.getTransformInGround(state).R());
                
                % gravity vector expressed in own body frame 
                grav_B = utility.osimVec3ToArray(ground.expressVectorInAnotherFrame(state,grav,body))';

                % Force of gravity applied at body origin in ground frame
                F_G = R_GB * mass * grav_B;
                % Moment of gravity about body origin in ground frame
                M_G = R_GB * cross(CoM, mass * grav_B);

                %append gravity moments and torques into GravF vector
                %indexed by mobod_index
                GravF_G = [GravF_G; M_G; F_G];
            end

            % System Jacobian relating spatial forces to generalized forces
            sysJ = Matrix(6*nbods, ncoords);
            smss = obj.model.getMatterSubsystem();
            smss.calcSystemJacobian(state, sysJ);
            sysJmat = utility.osimMatrixToArray(sysJ);

            % vector of generalized forces indexed by mobility (same as Q or state vector)
            Fgrav = sysJmat' * GravF_G; % (nu x 1)
            Fgrav(abs(Fgrav)<obj.zero_cut) = 0;
        end

        
        function Fbushing = calcBushingForces(obj, state)
            bushing_list = obj.bushingList;
            import org.opensim.modeling.*;

            nb = 16; % number of mobilized bodies where ground is mobod=0
            nu = 30; % number of mobilities in model

            %initializing empty vector of bushing forces 
            bushForceVec = zeros(nb, 6);
            nBushings = length(bushing_list);

            for kk = 1 : nBushings
            
                forceObj = obj.forceset.get(bushing_list{kk});
                %Have to convert to from a general osim Force Object to a Bushing
                %Object to access the BushingForce functions
                bushingObj = BushingForce.safeDownCast(forceObj);

                %MobilizedBodyIndex = 0 is ground, then one for each body after that
                mobod1 = bushingObj.getFrame1().getMobilizedBodyIndex();
                mobod2 = bushingObj.getFrame2().getMobilizedBodyIndex();  

                % calculate internal force based on deflection in current state
                deflection = utility.osimVectorToArray(bushingObj.computeDeflection(state));
                stiffness = zeros(1,6);
                stiffness(1:3) = utility.osimVectorToArray(bushingObj.get_rotational_stiffness());
                force = - deflection .* stiffness;

                %internal force on body 2
                fB2_q = force(1:3);
                fM_F = force(4:6);

                %get connected frames
                frame1 = bushingObj.getFrame1();
                frame2 = bushingObj.getFrame2();

                %get frame transforms in Ground frame
                X_GF = frame1.getTransformInGround(state);
                X_GM = frame2.getTransformInGround(state);

                %get transform from frame1 to frame 2
                % vec_F = X_FM * vec_M
                X_FM = frame2.findTransformBetween(state,frame1);

                dq = utility.osimVec3FromArray(deflection(1:3));
                N_FM = Rotation.calcNForBodyXYZInBodyFrame(dq);

                %calc the matrix relating q-space gen forces to a real-space moment vector.
                %We know qforce = ~H * moment. In that case, H would be N^-1, qforce =
                %~(N^-1) * moment so moment = ~N*qforce
                mB2_M = utility.osimMatrixToArray(N_FM)' * fB2_q'; %moment acting on body 2
                mB2_G = utility.osimMatrixToArray(X_GM.R().asMat33) * mB2_M; %moment on body 2 expressed in ground frame

                %Transform force from F frame to ground. This is the force to apply to body
                %2 at point OM; -f goes on body 1 at the same spatial location. Here we
                %actually apply it at OF so we have to account for the moment produced by
                %the shift from OM; 
                fM_G = utility.osimMatrixToArray(X_GF.R().asMat33) * fM_F';

                %Re-express local vectors in the Ground frame
                p_FM_G = utility.osimMatrixToArray(X_GF.R().asMat33) * utility.osimVec3ToArray(X_FM.p())';

                F2_G = [mB2_G; fM_G]; %(6x1) where (torques, forces)
                F1_G = [-(mB2_G + cross(p_FM_G',fM_G)'); -fM_G]; %(6x1) where (torques, forces)

                % Add in forces from this bushing into vector of spatial vectors
                % indexed by mobilized body index. 

                %NOTE: add 1 to mobod because of difference in indexing in OpenSim/C++ vs.
                %Matlab. Ground is mobod = 1 so nb is actually 16 even though we only
                %have 15 bodies.
                bushForceVec(mobod1+1,:) = bushForceVec(mobod1+1,:) + F1_G';
                bushForceVec(mobod2+1,:) = bushForceVec(mobod2+1,:) + F2_G';

            end

            % resize (nb x 6) array of bushing forces into (6*nb) vector Fbush_G
            bushForceVec = bushForceVec';
            Fbush_G = bushForceVec(:);

            % get system Jacobian relating spatial forces on 
            sysJ = Matrix(6*nb, nu);
            smss = obj.model.getMatterSubsystem();
            smss.calcSystemJacobian(state, sysJ);
            sysJmat = utility.osimMatrixToArray(sysJ);

            % vector of generalized forces indexed by mobility (same as Q or state vector)
            Fbushing = sysJmat' * Fbush_G; % (nu x 1)
            Fbushing(abs(Fbushing)<obj.zero_cut) = 0;

        end
        
        function axes = getBodyTransforms(obj, state)

            %transforms broken down into rotation matrix and translation vectors
            X_Gsp = obj.spinebody.getTransformInGround(state); %transformation matrix from spine body frame to ground frame
            axes.R_Gsp = round(utility.osimMatrixToArray(X_Gsp.R().asMat33()), 12); %rotation matrix component
            axes.p_Gsp = round(utility.osimVec3ToArray(X_Gsp.p()), 12); % translation component

            X_Gskull = obj.skullbody.getTransformInGround(state); %transform matrix from skull body to ground frame
            axes.R_Gfh = round(utility.osimMatrixToArray(X_Gskull.R().asMat33()), 12); %rotation component
            axes.p_Gfh = round(utility.osimVec3ToArray(X_Gskull.p()), 12) + obj.StationVector; %shift translation component to forehead from skull origin
            
            %need to get transfrom from spine_offset to obj.StationVector
        end
        
        function [laserIntersect, check] = laserWallLocation(obj, state)

            %laserMarker = model_constants.model.getMarkerSet().get('Laser');    
            laserLocVec3_Gnd = obj.laserMarker.getLocationInGround(state);
            laserLoc_Gnd = utility.osimVec3ToArray(laserLocVec3_Gnd);

            %spineObj = model_constants.model.getBodySet().get('spine');
            R_GSp = utility.osimMatrixToArray(obj.spinebody.getTransformInGround(state).R().asMat33);
            wall_norm = R_GSp(1,:);
            wall_dist = obj.distance_from_wall;

            n = wall_norm;
            V0 = (R_GSp * [wall_dist 0 0]')'; %wall is wall_dist meters away from GND Origin along spine x-axis 
            P0 = [0, 0, 0]; %GND Origin (same as spine origin)
            P1 = laserLoc_Gnd; %location at constant radius, not projected on wall yet

            u = P1-P0;
            w = P0 - V0;
            D = dot(n,u);
            N = -dot(n,w);
            check=0;
            if abs(D) < 10^-7        % The segment is parallel to plane
                if N == 0           % The segment lies in plane
                    check=2;
                    return
                else
                    check=0;       %no intersection
                    return
                end
            end

            %compute the intersection parameter
            sI = N / D;
            I = P0+ sI.*u;

            if (sI < 0 || sI > 1)
                check= 3;          %The intersection point  lies outside the segment, so there is no intersection
            else
                check=1;
            end

            laserIntersect = I;
        end
        
        % Calculates Factive and Fpassive from direct calls to OpenSim,
        % which calculates the pennation angle, active force length
        % multiplier, and passive force multiplier internally. 
        
%         function [Factive, Fpassive] = getMuscleForceScalingFactors(obj, state, muscleList)
%             for mm = 1 : length(muscleList)
%                 muscle = obj.muscleset.get(muscleList{mm});
%                 Fmax(mm) = muscle.getMaxIsometricForce();
%                 cosa(mm) = muscle.getCosPennationAngle(state);
%                 Fa(mm) = muscle.getActiveForceLengthMultiplier(state);
%                 Fp(mm) = muscle.getPassiveForceMultiplier(state);
%             end
%             
%             Factive = Fmax .* Fa .* cosa;
%             Fpassive = Fmax .* Fp .* cosa;
%             
%         end
        
        
    end
    

    methods (Static)
        
        function [Lfn, cosa] = calcFiberLengths(MTL, TSL, L0, a0)
    
            %Fiber length
            Lf = sqrt((L0 .* sin(a0)).^2 + (MTL - TSL).^2);
            Lfn = Lf ./ L0;

            %pennation angle = (Muscle Tendon Length - Tendon Slack Length) / Fiber
            %Length
            cosa = (MTL - TSL) ./ Lf;
        end
        
        function Factive = FaScalingFactor(Fmax, afl, Lfn, cosa)
    
            Factive = Fmax.* interp1(afl(:,1), afl(:,2), Lfn, 'pchip') .* cosa;

        end
        
        
        
        function Fpassive = FpScalingFactor(Fmax, pfl, Lfn, cosa)
    
            Fpassive = Fmax .* interp1(pfl(:,1), pfl(:,2), Lfn, 'pchip') .* cosa;

        end  
        
        %READ MUSCLE PARAMETERS FROM FILE  
        % Muscles must be listed in order of muscle index from osim Model
        % Uses Muscles.m class

        % column 1: index+1 of muscle in model
        % column 2: max isometric force (Fmax)
        % column 3: optimal fiber length (L0)
        % column 4: tendon slack length (TSL)
        % column 5: pennation angle at optimum length (a0)

        % file_path =
        % 'C:\Users\16179\Dropbox\Research\Projects\computational_modeling\NeckSimProject\OpenSim_Neck\muscle_parameters.xlsx';
        function mmParams = readMuscleParamsFromFile(file_path)

            data = xlsread(file_path);

            % for full set of 98 muscles
%             for ii = 1 : 98
%                 mmParams.Name(ii,:) = Muscles.getMuscleStringFromIndex(data(ii,1));
%             end
            
            %for reduced set of muscles (70)
            for ii = 1 : 70
                mmParams.Name(ii,:) = Muscles_Reduced.getMuscleStringFromIndex(data(ii,1));
            end

            mmParams.Fmax = data(:,2);
            mmParams.L0 = data(:,3);
            mmParams.TSL = data(:,4);
            mmParams.a0 = data(:,5);

        end
           
    end
    
end