%% RUN EXTRACTION AND FAS IN ONE DYNAMIC SCRIPT

% must have 'postures' array in workspace of correct number of columns to
% match the DOF in selected model (model_source)

load('C:\Users\16179\Documents\GitHub\osim_model\savedData\SmMatrix.mat')

%% SETUP

% Output Folder
output_folder = 'savedData';
%write description of simulation here to save with parameters 
sim_description = 'unconstrained 9dof (3 joints). upright. Model parameters from hyoid model and updated with new Vasavada model.';
%for file_name to save
base_name = 'upright_9dof';

% OpenSim Model
model_path = 'C:\libs\Model\neckModel-9jnt-reduce.osim';
muscle_list = ModelConstants.MuscleList_reduced;
ind_coord_list = ModelConstants.CoordList9;
coord_list = ModelConstants.CoordList9;
unconstrain = 0; % no constraints exist in this model

% osim libraries
import org.opensim.modeling.*;

% Instantiate ModalConstants object
osim = osimClass(model_path);
osim = osim.setMuscleList(muscle_list);
osim = osim.setCoordList(coord_list);
osim = osim.setIndCoordList(ind_coord_list);

%location of point of reference on head
station_vec = [0 0 0];
osim = osim.setStationVector(station_vec);

%body orientation('upright', 'prone', 'supine', 'fwd_lean', 'back_lean')
body_orientation = 'upright';
osim.changeBodyOrientation(body_orientation)

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

mm_source = 1;

switch mm_source
    case 1
        mmConstants = osim.getMuscleConstants();
    case 2
        muscle_param_file_path = ...
        'C:\Users\16179\Dropbox\Research\Projects\computational_modeling\NeckSimProject\OpenSim_Neck\model_params\conditions\mm_noMFSSC.xlsx';
        mmConstants = osim.readMuscleParamsFromFile(muscle_param_file_path);
    case 3
        %nothing. upload muscle constants from already created structure in
        %scaleGroupMuscleForces.m script
    otherwise
        error('mm_source must be set to 1 2 or 3');
end


%% neck structure

% neutral posture
posture = zeros(1,9); 
neck = NeckParams(posture, mmConstants, osim);

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

   
    
    