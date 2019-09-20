%% DEMONSTRATION

% --------------
%% OpenSim Visualizer

import org.opensim.modeling.*;
model_path = 'C:\libs\Model\neckModel-9jnt-reduce.osim';
%load Sm and postures structures
load('C:\Users\16179\Dropbox\Research\Projects\computational_modeling\NeckSimProject\Code\Code_Clean\savedData\3jnt_mmParams\Sm_simp70to24mm.mat')
load('C:\Users\16179\Dropbox\Research\Projects\computational_modeling\NeckSimProject\Code\Code_Clean\savedData\3jnt_mmParams\postures9_sag.mat')
% Instantiate osimClass object

osim = osimClass(model_path);
osim = osim.setMuscleList(ModelConstants.MuscleList_reduced);
osim = osim.setCoordList(ModelConstants.CoordList9);
osim = osim.setIndCoordList(ModelConstants.CoordList9);
osim = osim.setStationVector([0 0.2 0]); %top of head (load cell from Fice et al.)

osim.changeBodyOrientation('upright'); %('upright', 'prone', 'supine', 'fwd_lean', 'back_lean')

state = osim.model.getWorkingState();
pp=1;

%% Open and set up visualizer window 
visObj = vis(osim.model);

%% CHANGE POSTURE
%fwd head: 9, 17
%neutral: 1; flexed: 2; extended: 65
pp = 1;
visObj.updatePosture(postures(pp,:));

%% CAMERA front view
cam_pos = [10,0,0];
visObj.updateCameraPosition(cam_pos);

%% CAMERA back view
cam_pos = [-10,0,0];
visObj.updateCameraPosition(cam_pos);

%% CAMERA side view
cam_pos = [0,0,10];
visObj.updateCameraPosition(cam_pos);

%% CAMERA top view
cam_pos = [0,10,0];
visObj.updateCameraPosition(cam_pos);

%% CAMERA LOOK AT SKULL
skull_pos = visObj.updateLookAtSkull();

%% UPDATE MATLAB PLOT VIEW
camup([0 1 0]);
campos(cam_pos);
camtarget(skull_pos);
%camva(view_angle);
axis vis3d
%% Setup Plot
[fig,ax] = plotTools.setupNewFig([1,1]);

%% Plot Axes
axes = FAS.ppParams(pp).axes;
X_GF = utility.transformMatrix(axes.R_Gfh, axes.p_Gfh);
plotTools.plot_axes(X_GF, 1.2, fig, ax(1,1));

pp_str = plotTools.postureString(postures, pp);
fig_name = sprintf('Axes Plot for %s',pp_str);
title(fig_name); 


%% 
accel = FAS.Posture(pp).accel';
origin = FAS.ppParams(pp).axes.p_Gfh;
%% PLOT Convex Hull
accel_s = utility.scaleVecs(accel,1);
plotTools.plotConvexHull(accel_s,origin,fig,ax);

%% 
[x,y,z] = utility.mat2XYZ(accel);
[mag,uv] = utility.getMagAndUV(x,y,z,1);
plotTools.plotVectorMap(uv,mag,origin,fig,ax);


%%

plotTools.plot_vectors(x,y,z,origin,fig,ax)


%% CHANGE POSTURE AND CALC CONV HULL VOLUME
pp = 12;
visObj.updatePosture(postures(pp,:));
accel = FAS.Posture(pp).accel';
cVol = utility.computeCHullVolume(accel);
disp(cVol)