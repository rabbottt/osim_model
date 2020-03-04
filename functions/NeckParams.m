

function Neck = NeckParams(postures, mmConstants,osim)

    %import osim libraries
    import org.opensim.modeling.*;

    osimModel = osim.model;

    nPostures = size(postures,1);
    fprintf('Extracting parameters for %d postures.\n', nPostures);
    
    modConstants = ModelConstants;
    Neck.mmConstants = mmConstants;

    % repeat for each posture
    for pp=1 : nPostures
        
        fprintf('Posture %d / %d \n', pp, nPostures);
        
        posture = postures(pp,:);

        %reinitialize state to default and assemble so position constraints are
        %satisfied
        state = osimModel.initializeState();

        % Set model to specified posture
        osim.updateCoordinates(state, posture); 

        % realize state to position stage and update state of all muscles
        osimModel.realizePosition(state);
        osimModel.equilibrateMuscles(state);

        %Get updated vector of Qs at current state
        qVector = osim.getQVector(state);        
        
        % get the muscle tendon length from OpenSim based on current state
        MTL = osim.calcMTL(state);

        % Compute muscle fiber lengths assuming rigid tendon. Don't need to save.  
        [Lfn, cosa] = osimClass.calcFiberLengths(MTL, mmConstants.TSL, mmConstants.L0, mmConstants.a0);

        % Active muscle force scaling factor
        Neck.Params(pp).Factive = osim.FaScalingFactor(mmConstants.Fmax, modConstants.afl, Lfn, cosa);

        % Passive muscle force scaling factor
        Neck.Params(pp).Fpassive = osim.FpScalingFactor(mmConstants.Fmax, modConstants.pfl, Lfn, cosa);

        
        %Calculate Moment Arms in current state, returns (n x m) matrix
        %MAY MOVE TO mmDepNeck.Params
        Neck.Params(pp).R = osim.calcMomentArms(state);

        % %Get the inverse of the mass matrix, returns (n x n) matrix
        Neck.Params(pp).Minv = osim.getMinv(state);

        %Get Kinematic Jacobian (3 x n). 
        Neck.Params(pp).Jacobian = osim.getJacobian(state);
        
        % % % % % % % % % % %
        % % % TEMP DEBUG % % %
        % % % % % % % % % % %
        
        % Trying out different Jacobian to calculate Forces
        Neck.Params(pp).Jframe = osim.getFrameJacobian(state);        
        Neck.Params(pp).Jsys = osim.getSysJacobian(state);
        
        % % % % % % % % % % %
        % % % END DEBUG % % % 
        % % % % % % % % % % %
        
        %Get G: (n x 1) joint torque due to gravity
        

        Neck.Params(pp).Fgrav = osim.getGravityForces(state); 
        
        try
        % Get generalized forces due to bushing forces at joints (n x 1) vector
        Neck.Params(pp).Fbushing = osim.calcBushingForces(state);
        catch
            disp('No bushing forces calculated.')
        end

        %location of intercept of laser with wall. Wall is oriented so that it
        % is normal to the torso (if sitting upright, wall is upright; if lying
        % prone, wall is the floor). 
        [laserLoc, ~] = osim.laserWallLocation(state);

        %set up axes structure for plotting
        axes = osim.getBodyTransforms(state);
                
        %PostureNeck.Params.m class to save posture details
        Neck.ppParams.posture = posture;
        Neck.ppParams.joint_angles = qVector;
        Neck.ppParams.laser_location = laserLoc;
        Neck.ppParams.axes = axes;
    end
    
   
    Neck.postures = postures;
    Neck.muscleList = string(osim.muscleList);
    
    fprintf('Parameter extraction completed. Parameters are located in Neck structure.\n');

end