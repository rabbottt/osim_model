classdef utility
    properties
    end
    
    methods (Static)
        
        % create N unit vectors of r=1 evenly distributed over sphere
        function unitVecs = unit_sphere(N)    
            r = 1;
            count = 0;
            a = 4 * pi() * r^2 / N;
            d = sqrt(a);
            m_theta = round(pi()/d);
            d_theta = pi() / m_theta;
            d_phi = a / d_theta;

            for m = 0: m_theta - 1
                theta = pi() * (m + 0.5) / m_theta;
                m_phi = round(2 * pi() * sin(theta) / d_phi);
                for n = 0 : m_phi - 1
                    phi = 2 * pi() * n / m_phi;
                    count = count + 1;
                    UV(count,:) = r * [sin(theta) * cos(phi), sin(theta) * sin(phi), cos(theta)];
                end
            end
            unitVecs = round(UV, 4);
        end
        
        
        function Tmat = transformMatrix(R, p)

            Tmat = ...
                [ R(1,1), R(1,2), R(1,3), p(1); ...
                  R(2,1), R(2,2), R(2,3), p(2); ...
                  R(3,1), R(3,2), R(3,3), p(3); ...
                  0     , 0     , 0     , 1    ];

        end
        
        
        function deg = rad2deg(radians)
            deg = 180 * radians / pi();
        end
        
        function radians = deg2rad(deg)
            radians = deg * pi() / 180;
        end
        
        
        function crossmatrix = skewmat(v)

            crossmatrix = ...
                [0     -v(3)   v(2); ...
                 v(3)   0     -v(1); ...
                -v(2)   v(1)   0   ];

        end
        
        function [x, y, z] = mat2XYZ(column_matrix)

            if size(column_matrix,2) ~= 3
                if size(column_matrix,1) == 3
                    column_matrix = column_matrix';
                    warning('Input should be size nx3! Transposed matrix to match. Check that this is okay!')
                else
                    error('Input must be size nx3! Try again.')
                end
            end

            x = column_matrix(:,1);
            y = column_matrix(:,2);
            z = column_matrix(:,3);

        end
        
        function [x,y,z] = UVmag2XYZ(uv, mag)
            % check inputs
            if size(uv, 2) ~= 3
                error('check inputs! uv must be a matrix with 3 columns (corresponding to x, y, z components)');
            end    
            if size(uv, 1) ~= length(mag) 
                error('check inputs! the number of unit vectors and magnitudes must be equal.')
            end

            x = mag .* uv(:,1);
            y = mag .* uv(:,2);
            z = mag .* uv(:,3);

        end
        
        %compare to below commented out version
        function scaled_vecs = scaleVecs(vecs, scale_to)
    
            max_length = max(vecnorm(vecs,2,2));
            scaled_vecs = vecs * scale_to / max_length;

        end
        
%         function scaled_vec = scaleVec(vec, scale_to)
%             
%             %what about negative numbers??
%             if min(vec) < 0
%                 warning('Input vec contains negative numbers. Will only scale relative to maximum!')
%             end
%             
%            
%             %find maximum value in input vector 
%             max_value = max(vec);
%             
%             % scale all values in vec from [0 1], then multiply by scale_to
%             % to scale from [0 scale_to]
%             scaled_vec = vec / max_value * scale_to;
%         end
        
        function [mag, uv] = getMagAndUV(x, y, z, norm_max)
            
            %check that length of vectors x y and z are the same
            if length(x) ~= length(y) || length(x) ~= length(z)
                fprintf('length of x, y, and z must be equal.\n')
                return
            end

            % check number of inputs
            if nargin < 3 || nargin > 4 
                warning('Check Inputs: Wrong number of inputs to getMagAndUV! Womp womp 0_o');
                return
            end

            % build vector of scalar magnitudes and array of unit vectors
            mag = zeros(length(x), 1);
            uv = zeros(length(x), 3);

            % build arrays of mag: magnitude, and uv: unit vectors
            for vv = 1 : length(x)
                mag(vv) = norm([x(vv), y(vv), z(vv)]);
                
                %to avoid dividing by zero
                if abs(mag(vv)) < 10^-12
                    uv(vv,:) = [0, 0, 0];
                    warning('vector %d had too small of a magnitude to assign a direction. It was set to [0, 0, 0]',vv);
                else
                    uv(vv,:) = [x(vv), y(vv), z(vv)] / mag(vv);  
                end
            end

            % if a value was given to normalize against, then normalize mag
            if nargin == 4                
                % normalize vector magnitudes to be in range [0, norm_max]
                [magMax, ~] = max(mag);
                mag = mag / magMax * norm_max;                 
            end

        end
        
        % use this to find the posture index of a posture with: 
        %       ppIndex = findVecInMatrix(postures, postures);
        function rowIndex = findVecInMatrix(matrix,vector)

            vec_length = size(vector,2);
            numRows = size(matrix,2);

            if vec_length ~= numRows
                error('Check Inputs! The length of the vector (%d) must be equal to the length of a row of the matrix (%d)', vec_length, numRows);        
            end

            rowIndex = find(ismember(matrix,vector,'rows'));

        end
        
        function cHullVol = computeCHullVolume(column_vectors)
            DT = delaunayTriangulation(column_vectors);
            [~,cHullVol] = convexHull(DT);
        end
        
        function array = osimMatrixToArray(p)
            % import Java Libraries
             import org.opensim.modeling.*
            
             % Check the input type, will work for any of these
            if strcmp(class(p), 'org.opensim.modeling.Matrix') ...
                    || strcmp(class(p), 'org.opensim.modeling.Rotation') ...
                    || strcmp(class(p), 'org.opensim.modeling.Mat33') ...
                    || strcmp(class(p), 'org.opensim.modeling.Mat34') ...
                    || strcmp(class(p), 'org.opensim.modeling.Mat44')...

                 % Convert the input Opensim Matrix to a Matlab Array
                 size = [nrow(p),ncol(p)];
                 array = zeros(size);
                 for rr = 1 : size(1)
                     for cc = 1 : size(2)
                         array(rr,cc) = p.get(rr-1,cc-1);
                     end
                 end
            else
                error('Input must be org.opensim.modeling.Matrix, Rotation, Mat33, Mat34, or Mat44! 0_o ') 
            end
        end
        
        
        function vec = osimVec3FromArray(p)
            % import Java Libraries
             import org.opensim.modeling.*
            
             if strcmp(class(p),'double')
                % Check the size of the input vector is 1x3.
                if length(p) == 3 && (ismatrix(p) || iscolumn(p))
                    
                    % Convert the input vector to an OpenSim Vec3.
                    vec = Vec3( p(1), p(2), p(3) );
                    
                else
                    error('Input needs to be a vector of length 3')
                end  
            else
                error('Incorrect class input. Must be type double') 
            end
        end
        
        function rotation = osimRotationFromArray(array)
            if strcmp(class(array),'double')
                if size(array) == [3 3]
                    
                    % import Java Libraries
                    import org.opensim.modeling.*
                    rotation = Rotation(); 
                    for i = 1 : 3
                        for j = 1 : 3    
                            rotation().set(i-1,j-1,array(i,j));
                        end
                    end
                    
                else
                    error('Input must be a 3x3 matrix of doubles! 0_o ')
                end
            else
                error('Input must be a 3x3 matrix of doubles! 0_o ')
            end
        end
        
        function array = osimVectorToArray(p)
            % import Java Libraries
             import org.opensim.modeling.*
            % Check the input type
            if (strcmp(class(p), 'org.opensim.modeling.Vector') || ...
                    strcmp(class(p), 'org.opensim.modeling.Vec6') || ...
                    strcmp(class(p), 'org.opensim.modeling.Vec3'))
                
                % Convert the input Opensim Vec3 to a Matlab Vector
                array = zeros(1,size(p));
                for pp = 1 : size(p)
                   array(pp) = p.get(pp-1);
                end
            else
                error('Input must be of class org.opensim.modeling.Vector, Vec6, or Vec3') 
            end
        end

        
        function vec = osimVec3ToArray(p)
            % import Java Libraries
             import org.opensim.modeling.*
            % Check the input type
            if strcmp(class(p), 'org.opensim.modeling.Vec3')
                 % Convert the input Opensim Vec3 to a Matlab Vector
                 vec = [p.get(0) p.get(1) p.get(2)];
            else
                error('Input must be of class org.opensim.modeling.Vec3') 
            end
        end
        
    end
    
end