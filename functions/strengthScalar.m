
% strengthScalar.m 
% 
% Author: Rebecca Abbott 1/23/2019
% Based off of strengthScalar.m from Dan Lichtwark available under matlab
%       examples distributed with OpenSim
% 
%  

function strengthScalar(scaleFactor, muscleList, model_in, model_out)
    
    % Program to change scale strength of muscles and save to new .osim 
    % 
    % Inputs:   scaleFactor (double) - amount to scale muscle forces
    %           muscleList (list of strings) - muscles to scale (must be
    %                                           exact names)
    %           model_in (string) - existing model path and file name
    %           model_out (string) - new model path and file name
    % 


    import org.opensim.modeling.*
    
    filepath = model_in;
    fileoutpath = model_out;

    % Create the original OpenSim model from .osim file
    model1 = Model(filepath);
    model1.initSystem;
    
    % Create a copy of original OpenSim model for modified model
    model2 = Model(model1);
    model2.initSystem;
    
    % Rename the modified model 
    model2.setName('modelModified');
    
    % Get set of muscles that are in the original model and new model
    muscles1 = model1.getMuscles();
    muscles2 = model2.getMuscles();
    
    % Get number of muscles that will be scaled
    nMuscles = length(muscleList);
    
    for i = 1 : nMuscles
        
        currentMuscle = muscles1.get(muscleList(i));
        
        newMuscle = muscles2.get(muscleList(i));
        
        % calc new muscle force by multiplying current muscle max force by
        % the scale factor
        newMuscle.setMaxIsometricForce(currentMuscle.getMaxIsometricForce()*scaleFactor);
        
    end
    
    %save the updated model to an osim xml file
    
    model2.print(fileoutpath)
    disp(['The new model has been saved at ' fileoutpath]);

end
