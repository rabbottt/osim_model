%% RUN EXTRACTION AND FAS IN ONE DYNAMIC SCRIPT

% must have 'postures' array in workspace of correct number of columns to
% match the DOF in selected model (model_source)

%% SETUP

% Output Folder
output_folder = 'savedData\3jnt_upright';
%write description of simulation here to save with parameters 
sim_description = 'unconstrained 9dof (3 joints). prone. top of head. Model parameters from hyoid model and updated with new Vasavada model.';
%for file_name to save
base_name = 'upright_9dof'; %CHANGE THIS BASED ON POSTURE CONFIGURATION

% set model case
model_source = 6;
switch model_source
    case 1 % original with coupled vertebral motion (6DOF)
        model_path = 'C:\libs\Model\neckModel.osim';
        muscle_list = ModelConstants.MuscleList;
        ind_coord_list = ModelConstants.CoordList6;
        coord_list = ModelConstants.CoordList24;
        unconstrain = 0;
    case 2 % original with uncoupled vertebrae (24DOF)
        model_path = 'C:\libs\Model\neckModel.osim';
        muscle_list = ModelConstants.MuscleList;
        ind_coord_list = ModelConstants.CoordList24;
        coord_list = ModelConstants.CoordList24;
        unconstrain = 1;
    case 3 % 6 joint, all other vertebrae welded
        model_path = 'C:\libs\Model\neckModel-6jnt.osim';
        muscle_list = ModelConstants.MuscleList;
        ind_coord_list = ModelConstants.CoordList6;
        coord_list = ModelConstants.CoordList6;        
        unconstrain = 0; %no constraints exist in this model
    case 4 % 9 joint, all other vertebrae welded
        model_path = 'C:\libs\Model\neckModel-9jnt.osim';
        muscle_list = ModelConstants.MuscleList;
        ind_coord_list = ModelConstants.CoordList9;
        coord_list = ModelConstants.CoordList9;        
        unconstrain = 0; %no constraints exist in this model
    case 5 % only couple upper c-spine, uncouple lower
        model_path = 'C:\libs\Model\neckModel.osim';
        muscle_list = ModelConstants.MuscleList;
        ind_coord_list = ModelConstants.CoordList6;
        coord_list = ModelConstants.CoordList24;
        unconstrain = 1; % will only unconstrain the lower c-spine though! 
        constraint_indices = 0:14; %all constraints except for c0-c1 
    case 6 % 9 joint with reduced muscles
        model_path = 'C:\libs\Model\neckModel-9jnt-reduce.osim';
        muscle_list = ModelConstants.MuscleList_reduced;
        ind_coord_list = ModelConstants.CoordList9;
        coord_list = ModelConstants.CoordList9;
        unconstrain = 0; %no constraints exist in this model..
           
end



% osim libraries
import org.opensim.modeling.*;

% Instantiate ModalConstants object
osim = osimClass(model_path);
osim = osim.setMuscleList(muscle_list);
osim = osim.setCoordList(coord_list);
osim = osim.setIndCoordList(ind_coord_list);

%location of point of reference on head
skull_station = 'top_of_head';
switch skull_station
    case 'forehead'
        station_vec = [ 0.1 0.1 0 ]; %forehead
    case 'top_of_head'
        station_vec = [0 0.2 0]; %top of head (load cell from Fice et al.)
    otherwise
        station_vec = [0 0 0];
        disp('you did not select a station vector on the skull, using (0 0 0)')
end
osim = osim.setStationVector(station_vec);


%body orientation('upright', 'prone', 'supine', 'fwd_lean', 'back_lean')
body_orientation = 'upright';
osim.changeBodyOrientation(body_orientation)

% to enforce coupler constraints (6DOF) choose 1
% to not use constraints (24DOF) choose 0
% default is for constraints to be enforced.
if unconstrain == 1 
    if exist('constraint_indices', 'var') % if constraint_indices has been defined
        osim.setEnforceConstraints(0, constraint_indices);
    else
        osim.setEnforceConstraints(0)
    end
end

%save settings in setup structure
setup.unconstrain = unconstrain;
setup.station_vec = station_vec;
try
setup.body_orientation = body_orientation;
catch
end
setup.muscle_list = muscle_list;
setup.coord_list = coord_list;
setup.model_path = model_path;


%% COND STRUCTURE 

% mm_source = 1 for muscle parameters from normal model .osim file
% mm_source = 2 for muscle paramters from excel spreadsheet (no multif)
% mm_source = 3 for muscle parameters scaled elsewhere

mm_source = 3;

switch mm_source
    case 1
        %mmConstants = osim.getMuscleConstants();
        mmConstants = osim.getMuscleConstants();
    case 2
        muscle_param_file_path = ...
        'C:\Users\16179\Dropbox\Research\Projects\computational_modeling\NeckSimProject\OpenSim_Neck\model_params\conditions\mm_noMFSSC.xlsx';
        mmConstants = osim.readMuscleParamsFromFile(muscle_param_file_path);
    case 3
        %nothing. upload muscle constants from already created structure in
        %scaleGroupMuscleForces.m script
    otherwise
        error('mm_source must be set to 1 or 2');
end


%% neck structure

neck = NeckParams(postures,mmConstants,osim);

neck.info.description = sim_description;
neck.info.model_path = model_path;
neck.setup = setup;

%% SAVE NECK STRUCTURE (PARAMETERS)

try
    file_name = sprintf('Params_%s_%s.mat', base_name, datestr(now,'mm-dd-yyyy_HH-MM'));
    full_file_name = fullfile(output_folder, file_name);
    save(full_file_name, 'neck');
    
    disp(['The neck Param structure has been saved at ' full_file_name]);
    
catch
    warning('Unable to save neckParams to .mat file');
end



%% RUN FAS OPTIMIZATIONS

unitvecs = utility.unit_sphere(100);

FAS_version = 'reduced_mm';
%changed NeckFAS to not include Fb and Fp
switch FAS_version
    case 'all_mm'
        FAS = NeckFAS2(neck,unitvecs); %full 98 muscles
    case 'reduced_mm'
        options.Sm = Sm;
        FAS = NeckFAS2(neck,unitvecs,options); % simplify to smaller set of muscles using Sm matrix
        FAS.Sm = Sm;
    otherwise 
        warning('Select a version of FAS.')
end
FAS.info.mm_set = FAS_version;

%% run FWS
unitvecs = [1,0,0,0,0,0;0,1,0,0,0,0;0,0,1,0,0,0;0,0,0,1,0,0;0,0,0,0,1,0;0,0,0,0,0,1];
options.Sm = Sm;
FWS = NeckFWS(neck,unitvecs,options);

%% SAVE FAS STRUCTURE

try
%     Save FAS structures into FAS mat file in savedData folder
    file_name = sprintf('FASall_%s_%s.mat', base_name, datestr(now,'mm-dd-yyyy_HH-MM'));
    full_file_name = fullfile(output_folder, file_name);
    save(full_file_name, 'FAS');
    
    disp(['The FAS structure has been saved at ' full_file_name]);
catch
    warning('Unable to save FAS to .mat file');
end

%%

   
    
    