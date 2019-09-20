

load('C:\Users\16179\Dropbox\Research\Projects\computational_modeling\NeckSimProject\Code\Code_Clean\savedData\unconstr_9dof\sampling\actspace24mm_pp1_againstgrav13.mat')

%% using interactive parallel coordinate visualization toolbox
addpath('C:\Users\16179\Dropbox\Research\Projects\computational_modeling\NeckSimProject\Code\toolbox\interactiveParallelCoordinates')

mmI = [1 3 5 7 9 11 13 15 17 19 21 23];
lengthX = size(Act(1).X,1);

actX = Act(1).X(10:100:lengthX,mmI);

include_costs = 1;

switch include_costs
    case 0 %don't include cost functions
        x_min_max = [zeros(1,size(actX,2)); ones(1,size(actX,2))];
        ax_names = {'Deep Low Ext R', 'Deep Low Flex R', ...
            'Deep Mid Ext R', 'Deep Mid Flex R', ...
            'Deep Upper Ext R', 'Deep Upper Flex R', ...
            'Intermed Low Ext R', 'Intermed Low Flex R', ...
            'Intermed Upper Ext R', 'Intermed Upper Flex R',...
            'Global Ext R', 'Global Flex R'};
        
    case 1 %do include cost functions

        cost1 = ActSpaceSampling.computeCost(actX,'1norm');
        cost2 = ActSpaceSampling.computeCost(actX,'2norm');
        cost3 = ActSpaceSampling.computeCost(actX,'maxnorm');
    
        c1min = min(cost1);    
        c1max = max(cost1);
        c2min = min(cost2);
        c2max = max(cost2);
        c3min = min(cost3);
        c3max = max(cost3);
        
        actXcost = [actX cost1 cost2 cost3];
        x_min_max = [zeros(1,size(actX,2)) c1min c2min c3min; ones(1,size(actX,2)) c1max c2max c3max];
        ax_names = {'Deep Low Ext R', 'Deep Low Flex R', ...
            'Deep Mid Ext R', 'Deep Mid Flex R', ...
            'Deep Upper Ext R', 'Deep Upper Flex R', ...
            'Intermed Low Ext R', 'Intermed Low Flex R', ...
            'Intermed Upper Ext R', 'Intermed Upper Flex R',...
            'Global Ext R', 'Global Flex R', ...
            'cost1', 'cost2', 'cost3'} ;
end


startParCoords(actXcost, ax_names, x_min_max);
% make format %1.1f