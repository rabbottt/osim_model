classdef vis < handle
    
    properties
        model % reference to osim model
        visualizer %osim reference to OpenSim visualizer object
        simTKvis % osim reference to simTK visualizer object
        camera_target = [5, 0, 0]; % initial target for camera view
        camera_position = [8, 0, 0];
        field_of_view = 0.1; % camera viewing angle, similar to zoom
        up_axis % (+) y-axis (set in constructor to access Vec3 object with osim library)
        skullObj % Vec3 [x,y,z] position of skull frame in ground frame
        transform % null osim Transform() object
        
              
    end
    
    properties (Constant)
        indCoordList = ModelConstants.CoordList9; % list of independent coordinate names
        background_type = 2; %enum: solid color = 2; ground and sky = 1;
        background_color = [0, 0, 0]; %black background
        model_path = 'C:\libs\Model\neckModel.osim'; %take out after debug
        fh_skull = [0.1, 0.1, 0];
    end
    
    methods
        %constructor
        function obj = vis(modelObj)
            %import opensim libraries
            import org.opensim.modeling.*;
            
            %create model object for this instance of class
            obj.model = modelObj;
            
            %tell opensim to use the visualizer. Must do initSystem before
            %you can view the model in the visualizer
            obj.model.setUseVisualizer(1);
            state = obj.model.initSystem;
            
                       
            
            %SET UP THE VISUALIZER WINDOW
            
            %create writable visualizer object 
            obj.visualizer = obj.model.updVisualizer();
            %get reference to underlying simbody visualizer class
            obj.simTKvis = obj.visualizer.getSimbodyVisualizer();
            %set background color to solid (enum = 2) and black [0, 0, 0];
            obj.simTKvis.setBackgroundTypeByInt(obj.background_type);  
            color_vec = utility.osimVec3FromArray(obj.background_color);
            obj.simTKvis.setBackgroundColor(color_vec);%black background_color
            
            %create a null simTK Transform() object to use in later methods
            obj.transform = Transform().setToZero();
            
            obj.updateCameraPosition(obj.camera_position);
            % set the up direction for the camera to the (+) y axis            
            obj.up_axis = Vec3(0, 1, 0);
            %get a reference to the skull body frame to get position in
            %ground coordinates. This will be used as a target for the
            %camera to point at. 
            obj.skullObj = obj.model.getBodySet().get('skull');
            obj.updateLookAtSkull();  
            obj.updateFieldOfView(obj.field_of_view); 
            
            %Evaluate the geometry needed to visualize the given state and 
            %use it to generate a new image in the Visualizer window.
            obj.visualizer.show(state); 
                      
            
        end   
                
        
        function [] = updatePosture(obj, posture)
            % number of independent coordinates in the model (9)
            nCoords = length(obj.indCoordList);
            % number of angles from posture input (9)
            nAngles = length(posture);
            
            % get a reference to the current state of the model
            state = obj.model.getWorkingState();
            
            % check that the input posture vector fully defines the
            % coordinates
            if nCoords ~= nAngles
                warning('List of coordinate names must be same length as number of joint angles.  Check your inputs! 0_o');
                return
            end         
            
            %get writable reference to the set of model coordinates
            coordset = obj.model.updCoordinateSet();
            %set each independent coordinate to the angle specified by the
            %'posture' input in radians. 
            for jj=1 : nCoords
                coord = coordset.get(obj.indCoordList{jj});
                % enforce constraints when the final value is set 
                coord.setValue(state, utility.deg2rad(posture(jj)), double(jj==nCoords));
            end            
                        
            obj.visualizer.show(state)
            
            %DOESN'T WORK
            %update window title to display current posture             
%           window_title = sprintf('Posture: [%d, %d, %d, %d, %d, %d, %d, %d, %d]', ...
%                 posture(1), posture(2), posture(3), posture(4), posture(5), ...
%                 posture(6), posture(7), posture(8), posture(9));
%             obj.simTKvis.setWindowTitle(toString(window_title));
        end       
        
        % 
        % Update view to point at 'camera_target'. 
        % INPUT: camera_target: 1x3 array of doubles [x, y, z] in ground
        %                       coordinates.
        % DEPENDENTS: utility class: osimVec3FromArray(array) 
        % 
        function [] = updateCameraTarget(obj, camera_target)
            % convert matlab array to Opensim Vec3
            target = utility.osimVec3FromArray(camera_target);
            % rotate camera to point at 'skull_loc' with camera up
            % direction set to 'up_axis'
            obj.simTKvis.pointCameraAt(target, obj.up_axis);            
        end
        
        % 
        % Update view to look at the origin of the skull frame
        % INPUTS: none
        % OUTPUTS: target: 1x3 array of doubles [x, y, z] in ground frame
        % DEPENDENTS: utility class, osimVec3ToArray(Vec3) method 
        %
        function target = updateLookAtSkull(obj)
            %get a reference to the current state of the model
            state = obj.model.getWorkingState();
            % get Vec3 position of skull frame in Gnd frame
            skull_loc = obj.skullObj.getPositionInGround(state); 
            % rotate camera to point at 'skull_loc' with camera up
            % direction set to 'up_axis'
            obj.simTKvis.pointCameraAt(skull_loc, obj.up_axis)
            % convert OpenSim Vec3 to matlab 1x3 array of doubles
            target = utility.osimVec3ToArray(skull_loc);   
            obj.visualizer.show(state); 
        end
        
        % 
        % Update the camera Field of View
        % INPUTS: field_of_view: double, similar to zoom where smaller
        %                           number is more zoomed in
        % OUTPUTS: none
        % DEPENDENTS: none
        % 
        function [] = updateFieldOfView(obj, field_of_view)
            obj.simTKvis.setCameraFieldOfView(field_of_view);
        end
        
        
        % 
        % Update the view to show all of the model geometry
        % INPUTS: none
        % OUTPUTS: none
        % DEPENDENTS: none
        %         
        function [] = showAllGeometry(obj)
            obj.simTKvis.zoomCameraToShowAllGeometry();
        end
        
        
        % 
        % Update the camera view by rotating camera from same position
        % INPUTS: R: 3x3 double array rotation matrix 
        % OUTPUTS: none
        % DEPENDENTS: utility class, osimRotationFromArray(R) method
        % 
        function [] = updateCameraRotation(obj, R)
            % convert from 3x3 double matlab array to OpenSim Rotation()
            R = utility.osimRotationFromArray(R);
            % Set the transform defining the orientation of the camera.
            obj.simTKvis.setCameraTransform(R);
        end
        
        % 
        % Update the camera view by moving the position of the camera
        % INPUTS: p: 1x3 double array [x,y,z] 
        % OUTPUTS: none
        % DEPENDENTS: utility class, osimVec3FromArray(p) method
        % 
        function [] = updateCameraPosition(obj, p)
            % convert from matlab array to OpenSim Vec3
            p = utility.osimVec3FromArray(p);
            % create an OpenSim transfrom with null R and translation set
            % to vector p
            T = obj.transform.setP(p);
            % Set the transform defining the position of the camera.
            obj.simTKvis.setCameraTransform(T);
        end
        
%         function decObj = drawRubberBandLines(obj,mobod1, station1, mobod2, station2, color)
%        
%          
%             decObj = obj.simTKvis.addRubberBandLine(mobod1, station1, mobod2, station2, DecorativeLine);
%         end


        function decFrame = drawAxisFrame(obj, mobod_index)
            import org.opensim.modeling.*;
            decFrame = DecorativeFrame; 
%             mobod_index = 14;
            decFrame.setBodyId(mobod_index); %14 for skull
            axis_length = 1;
            decFrame.setAxisLength(axis_length);
            axis_color = Vec3(0,1,0);
            decFrame.setColor(axis_color);
            decFrame.setLineThickness(10);
            %X_skullFH = Transform(Vec3(0.1,0.1,0));
            %decFrame.setTransform(X_skullFH); %transform from skull to forehead
            
            state = obj.model.getWorkingState();
            obj.simTKvis.drawFrameNow(state);

        end
        
        function arrow = drawArrow(obj,  mobod_index, origin, vector)
            import org.opensim.modeling.*;
            
            if size(origin) ~= size(vector)
                error('start_pt and vector must be 3x1 or 1x3 vectors of same size')
            end
            
            start_pt = utility.osimVec3FromArray(origin);
            end_pt = utility.osimVec3FromArray(origin + vector);
            tipLength = 0.35; %default from osim
            
            arrow = DecorativeArrow(start_pt, end_pt, tipLength);
            arrow.setBodyId(mobod_index);
            arrow.setColor(Vec3(1, 0, 0));
            %arrow.setRepresentation('DrawWireframe');
            enum_setting = arrow.getRepresentation().swigToEnum(2);
            arrow.setRepresentation(enum_setting);
            
            obj.simTKvis.addDecoration(mobod_index, Transform(), arrow);
            
            state = obj.model.getWorkingState();
            obj.simTKvis.drawFrameNow(state)
        end
        
        function rbIndex = createRubberBandLine(obj, mobod1, pt1, mobod2, pt2)
            import org.opensim.modeling.*;
            dec_line = DecorativeLine();
            
            dec_line.setColor(Vec3(0,1,1));
            dec_line.setLineThickness(5);
            
            pt1 = utility.osimVec3FromArray(pt1);
            pt2 = utility.osimVec3FromArray(pt2);
            rbIndex = obj.simTKvis.addRubberBandLine(mobod1, pt1, mobod2, pt2, dec_line);
            
            state = obj.model.getWorkingState();
            obj.simTKvis.drawFrameNow(state);
            
            
        end
        
        %pt2 is the relative to the frame of mobod2 defined when rubberband
        %line was created. May need to instead define all in ground
        %coordinates.. 
        function updRBL(obj, rbIndex, pt2)
            
            rbObj = obj.simTKvis.updRubberBandLine(rbIndex);
            state = obj.model.getWorkingState();
            % get Vec3 position of skull frame in Gnd frame
            skull_G = osimVec3ToArray(obj.skullObj.getPositionInGround(state)); 
            FH_skull = obj.fh_skull;
            fh_G = skull_G + FH_skull; % location of forehead in ground
            
            pt2 = fh_G + pt2; %offset vector by to start at origin
            
            rbObj.setPoint1(osimVec3FromArray(fh_G));
            rbObj.setPoint2(osimVec3FromArray(pt2));
            
            %may leave this out if called again later
            obj.simTKvis.drawFrameNow(state);
        end

    end
    
    methods (Static)
    end
    
end