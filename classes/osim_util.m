classdef osim_util
    
    methods (Static)
        
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

        
        function matlabcellarray = osimList2MatlabCell(model, classname)
            % osimList2MatlabCell()
            %   Convert an OpenSim list into a cell array of OpenSim
            %   references. The cell array can then be used to get
            %   references to individual components. 
            %
            % 
            %   % Get a cell array of references to all the bodies in a model
            %   references = osimList2MatlabCell(model,'Body')
            %   % Get the first body in the list.  
            %   Pelvis = references{1}
            %
            %   osimList2MatlabCell() uses the ComponentList() and Interator() classes 
            %   to iterate through a list to get the components. An example of using
            %   the iterator is below;
            %
            %   % Set a value for a property for all of the components of type 'Body'
            %   list = model.getBodyList();
            %   li = list.begin()
            %   while ~li.equals(list.end())
            %       li.next();
            %       li.setSomePropertyValue(val);
            %   end
            
            % Options for classname: 
                % 'Body', 'Frame', 'Joint', 'Millard2012EquilibriumMuscle',
                % 'ModelComponent', 'Muscle', 'Thelen2003Muscle'

            % import opensim libraries. 
            import org.opensim.modeling.*

            % Try the givenclassname
            try
                eval(['list = model.get' classname 'List();']);
                disp('List instantiation Successful');
            catch 
                error(['No list for get' classname 'List() found']);
            end

            % Get a reference to the component of interest
            matlabcellarray = {};
            li = list.begin();
            while ~li.equals(list.end())
                matlabcellarray{end + 1,1} = li.deref;
                li.next();
            end 

        end
        
        function matlabcellarray = osimSet2MatlabCell(model, classname)
            % BY RAB
            % osimSet2MatlabCell()
            %   Convert an OpenSim Set into a cell array of OpenSim
            %   references. The cell array can then be used to get
            %   references to individual components. 
            %
            %   % Get a cell array of references to all the bodies in a model
            %   references = osimSet2MatlabCell(model,'Body')
            %   % Get the first body in the list.  
            %   Pelvis = references{1}
            %
 
            % Inputs:   model
            %           class_name: string options-> 'Analysis','Body',
            %                               'Constraint','ContactGeometry', 
            %                               'Controller', 'Coordinate', 
            %                               'Force', 'Joint', 'Marker', 
            %                               'Muscle, 'Probe', 

            % import opensim libraries. 
            import org.opensim.modeling.*
                                 
            switch classname
                case 'Actuator'
                    list = model.getActuators();
                case 'Analysis'
                    list = model.getAnalysisSet();
                case 'Body'
                    list = model.getBodySet();
                case 'Constraint'
                    list = model.getConstraintSet();
                case 'ContactGeometry'
                    list = model.getContactGeometrySet(); 
                case 'Controller'
                    list = model.getControllerSet();
                case 'Coordinate'
                    list = model.getCoordinateSet();
                case 'Force'
                    list = model.getForceSet();
                case 'Joint'
                    list = model.getJointSet();
                case 'Marker'
                    list = model.getMarkerSet();
                case 'Muscle'
                    list = model.getMuscles();
                case 'Probe'  
                    list = model.getProbeSet();
                otherwise
                    error(['No method to get ' classname ' found']);
            end

            % get the size of the list
            try
                n = list.getSize();
            catch 
                error('Could not get the size using classname.getSize()')
            end

            % Get a reference to the component of interest
            matlabcellarray = {};

            for ii = 1 : n
                matlabcellarray{ii,1} = list.get(ii-1);
            end
            
        end

        
        % Convert Matlab Struct to OpenSim time Series Table
        %  Input is a Maltab stucture where data.label = nX1 or nX3 array
        %  eg s.LASI = [2 3 4; 4 5 6, ...
        %  One of the structures values MUST be a time vector and called 'time'
        %  Output is an OpenSim TimesSeriesTable

        % Written by: James Dunne, Tom Uchida, Chris Dembia, Ajay Seth,
        %                   Ayman Habib, Jen Hicks,Shrinidhi K. Lakshmikanth.

        function timeseriesosimtable = osimTableFromStruct(s)
            % import Java Libraries
            import org.opensim.modeling.*

            % Get all the data labels in the Structure
            labels = fieldnames(s);

            % Check for, save, then remove the time array from the structure
            % Get the index for time array
            tIndex = find(cellfun(@(s) contains('time', s),lower(labels)));
            % Check to see if the time exists
            if isempty(tIndex)
                error('"time" field required, none found')
            end
            % Get the time vector
            time = s.time();
            % Remove time from the struct
            s = rmfield(s,'time');
            % remove time from the labels
            labels(tIndex) = [];
            nfields = length(labels);

            % Check the structure for row and column length consistency across fields. 
            for i = 1 : nfields
                % For all fields in s, check that the array size is either nX1 or nX3,
                % other array sizes are unsupported. 
                [nRows,nCols] = size(s.(labels{i}));

                if i == 1
                    rowRef = nRows; colRef = nCols;
                    % Check that the first field has 1 or 3 Columns
                    if ~(colRef == 1 || colRef == 3)
                       error(['Data Columns must be 1 or 3, s.' labels{i} ' has ' num2str(colRef)])
                    end
                else
                    % Check that the current field has the same number of rows and
                    % columns as the reference. 
                    if rowRef ~= nRows || colRef ~= nCols
                        error('Array rows or columns are non-uniform across fields. Check input stucture for error')
                    end
                end
            end

            % Instantiate an empty OpenSim TimesSeriesTable()
            if colRef == 1
                timeseriesosimtable = TimeSeriesTable();
            else
                timeseriesosimtable = TimeSeriesTableVec3();
            end

            % Set the TimesSeriesTable() column names
            osimlabels =  StdVectorString();
            for i = 1 : nfields
                osimlabels.add( labels{i} );
            end
            timeseriesosimtable.setColumnLabels(osimlabels);

            % Build the TimesSeriesTable()
            if colRef == 1
                % Get an OpenSim Row Vector
                row = RowVector(nfields, NaN);
                % Fill row vector with data 
                for iRow = 1 : nRows 
                    for iCol = 1 : nfields 
                       % Get the row value from each field of the struct
                       row.set(iCol-1, s.(labels{iCol})(iRow) );
                    end
                    % Append the row vector to the opensim table
                    timeseriesosimtable.appendRow(iRow-1, row);
                end
            else
                % Get an OpenSim Row Vector
                row = RowVectorVec3(nfields);
                for iRow = 1 : nRows
                    % Create and fill a row of data
                    for iCol = 1 : nfields 
                        % Make a vec3 element from the rowdata
                        row.set(iCol-1, osimVec3FromArray(s.(labels{iCol})(iRow,:)));
                    end
                    % Append the RowVectorofVec3's to the opensim table
                    timeseriesosimtable.appendRow(iRow-1, row);
                end
            end

            % Set the Time Column values
            timeColumn = timeseriesosimtable.getIndependentColumn();
            for i = 1 : nRows 
                  timeColumn.set(i-1, time(i));
            end

        end
        
        % Convert Matlab Struct to OpenSim time Series Table
        %  Input is an OpenSim TimesSeriesTable or TimesSeriesTableVec3
        %
        %  Output is a Maltab stucture where data.label = nX1 or nx3 array
        %  eg structdata.LASI = [2 3 4; 4 5 6, ...

        % Author: James Dunne, Tom Uchida, Shrinidhi K. Lakshmikanth, Chris Dembia, 
        %         Ajay Seth, Ayman Habib, Jen Hicks, Apoorva Rajagopal.

        %
        function structdata = osimTableToStruct(osimtable)

            % Import Java libraries 
            import org.opensim.modeling.*

            % Type check the input variables 
            if strcmp( char(osimtable.getClass()), 'class org.opensim.modeling.TimeSeriesTableVec3')
                    vec3Table = true;

            elseif strcmp( char(osimtable.getClass()), 'class org.opensim.modeling.TimeSeriesTable')
                    vec3Table = false;

            else
                disp(['Input is of type ' char(osimtable.getClass()) ])
                error(['This function only converts TimeSeriesTable and TimeSeriesTableVec3']);
            end

            % get the number of data columns as label headers and data rows
            nLabels = osimtable.getNumColumns(); 
            nRows = osimtable.getNumRows();

            % pre-allocate data arrarys. 
            structdata = struct();
            if vec3Table == true
                dataArray = NaN([nRows 3]);
            else
                dataArray = NaN([nRows 1]);
            end

            % cycle through columns and rows to make new arrays, then addd then to struct. 
            for iLabel = 0 : nLabels - 1
                %     
                if vec3Table == true
                    % If the data is Vec3 type
                    for iRow = 0 : nRows - 1
                        dataArray(iRow+1,1) = osimtable.getDependentColumnAtIndex(iLabel).get(iRow).get(0);
                        dataArray(iRow+1,2) = osimtable.getDependentColumnAtIndex(iLabel).get(iRow).get(1);
                        dataArray(iRow+1,3) = osimtable.getDependentColumnAtIndex(iLabel).get(iRow).get(2);
                    end
                else
                    % If the data is double type
                    for iRow = 0 : nRows - 1
                        dataArray(iRow+1,1) = osimtable.getDependentColumnAtIndex(iLabel).getElt(iRow,0);
                    end
                end

                % Get the osim table column label
                col_label  = char(osimtable.getColumnLabels.get(iLabel));
                disp_label = col_label;
                % MATLAB variable names must start with a letter, can only contain letters,
                % digits, and underscores, and be under a maximum length
                if ~isvarname(col_label)
                    % Find any non-alphanumeric characters and replace with '_'
                    col_label(~(isstrprop(col_label,'alphanum'))) = '_';
                    % Check if first character is a letter, and append 'unlabeled' if not
                    if ~(isletter(col_label(1)))
                        col_label = ['unlabeled', col_label];
                    end
                    % Last check for too long a name 
                    % Have user input a new name, and make them keep doing it until the
                    % variable name is valid
                    while ~isvarname(col_label)
                        disp(['Error: ', disp_label ' is an invalid label name; enter a new name that:'])
                        disp('  - starts with a letter')
                        disp('  - contains only letters, numbers, and/or underscores (_)')
                        disp(['  - is no longer than ', num2str(namelengthmax), ' characters'])
                        col_label = input('New name (no quotes): ', 's');
                    end
                end
                % If the label has changed, inform the User.
                if ~strcmp(disp_label,col_label)
                    disp(['Illegal Column label. ' disp_label ' changed to ' col_label ]); 
                end
                % Add the label and data to the data struct
                structdata.(col_label) = dataArray;
            end

            % Get the time
            time = NaN([nRows 1]);
            % read time data from table
            for iRow = 0 : nRows - 1
                time(iRow+1,1) = osimtable.getIndependentColumn.get(iRow);
            end
            % add time field to structure
            [structdata.time] = time;

        end
        
        
    end
    
end
    