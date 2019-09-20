
classdef ActSpaceSampling

    properties
    end
    
    methods (Static)   
      
        function X = sampleActSpaceGrav(R, Fa, Fg, options)
            %Inputs: 
            %           R: Moment Arm Matrix
            %           Fa: active Force Multipliers (Fmax * F_length)
            %           Fg: generalized force (torques) due to gravity
           
            if isfield(options, 'tol')
                toln = options.tol;
            else
                toln = 10^-2; %default
            end
            
            if isfield(options, 'Sm')
                Sm = options.Sm;
            else
                Sm = 1; %use all muscles instead of grouping
            end
                        
            % - Fg = R * Fa * Sm * a
            L = R * Fa * Sm;
            G = - Fg;

            tol = toln * ones(length(G),1);
            %upper bound for specific direction
            A_specU = L;
            b_specU = G + tol;

            %lower bound for specific direction 
            A_specL = - L;
            b_specL = - G + tol;
           
            
            % sample activation space
            %combine inequalities for direction and muscle activation bounds
            %inequalities for muscle activations [0 1]
            nMuscles = size(L,2);
            A=[A_specU; A_specL; diag(ones(nMuscles,1)); -1*diag(ones(nMuscles,1))];
            b=[b_specU; b_specL; ones(nMuscles,1); zeros(nMuscles,1)];

            options.x0 = chebycenter(A,b);
            options.method = 'hitandrun';

            n=1000000;
            [X,~] = cprnd(n,A,b,options);

        end
        
        function X = sampleActSpaceAccel(posture_index, neckParams, options)
            %'accel': options.mode = 'accel': options.dir_index, options.intensity,
                    %options.FAS, options.Sm (optional)
            %'iso_force': options.mode = 'iso_force':
                    %options.direction_index, options.intensity,
                    %options.FFS(?), options.Sm (optional)                    
            %'wrench': options.mode = 'wrench': options.dir_index,
                    %options.intensity, options.FWS, options.Sm (optional)
            
            pp = posture_index;            
            space = options.mode;
            
            if isfield (options,'Sm')
                Sm = options.Sm;
            else
                Sm = 1;
            end

            %tolerance for matching task acceleration 
            %CHECK HOW THIS IS AFFECTED
            toln = 10^-2;             

            % Parameters from neckParams structure
            J = neckParams.Params(pp).Jacobian;
            Jf = neckParams.Params(pp).Jframe;
            R = neckParams.Params(pp).R;
            Fa = diag(neckParams.Params(pp).Factive);
            %Fp = Cond.Posture(pp).Fpassive;
            Fg = neckParams.Params(pp).Fgrav;
            %Fb = Base.Posture(pp).Fbushing;
            Minv = neckParams.Params(pp).Minv;
            
            switch space
                case 'accel'
                    nn = options.dir_index;
                    %choose ratio [0 1] of max acceleration to specify task (ex: p=0.5 will
                    %select 50% of max accel)
                    p = options.intensity;
                    
                    L = J * Minv * R * Fa * Sm; %(3x8)
                    %G = J * Minv * (R * Fp + Fb + Fg); %(3x1)
                    G = J * Minv * Fg;
                    
                    %get max acceleration found in linprog optimization
                    if isfield(options,'FAS')
                        a_max = options.FAS.Posture(pp).accel(:,nn);
                    elseif isfield(options,'max_accel')
                        if isvector(options.max_accel)
                            a_max = options.max_accel;
                        elseif ismatrix(options.max_accel)
                            a_max = options.max_accel(:,nn);
                        end
                    else
                        error('Must have options.FAS or options.max_accel field')
                    end
                    
                    tol = toln * ones(length(G),1);
                    %upper bound for specific direction
                    A_specU = L;
                    b_specU = p * a_max - G + tol;

                    %lower bound for specific direction 
                    A_specL = - L;
                    b_specL = - p * a_max + G + tol;
                    
                case 'wrench'
                    nn = options.dir_index;
                    %choose ratio [0 1] of max acceleration to specify task (ex: p=0.5 will
                    %select 50% of max accel)
                    p = options.intensity;               
                    
                    %get max wrench found in linprog optimization
                    if isfield(options,'FWS')
                        w_max = options.FWS.Posture(pp).wrench(:,nn);
                    elseif isfield(options,'max_wrench')
                        if isvector(options.max_wrench)
                            w_max = options.max_wrench;
                        elseif ismatrix(options.max_accel)
                            w_max = options.max_wrench(:,nn);
                        end
                    else
                        error('Must have options.FWS or options.max_wrench field')
                    end
                    
                    tol = toln * ones(size(Jf,2),1);
                    %upper bound for specific direction
                    A_specU = R * Fa * Sm;
                    b_specU = p * Jf' * w_max + tol;

                    %lower bound for specific direction 
                    A_specL = - R * Fa * Sm;
                    b_specL = - p * Jf' * w_max + tol;
                    
                    
                case 'iso_force' %need Fmax from FFS
                    Jtinv = pinv(J');
                    L = -Jtinv * R * Fa * Sm;
                    G = -Jtinv * Fg;
                    
                    p = options.intensity;
                    
                    tol = toln * ones(length(G),1);
                    %upper bound for specific direction
                    A_specU = L;
                    b_specU = p * Fmax - G + tol;

                    %lower bound for specific direction 
                    A_specL = - L;
                    b_specL = - p * Fmax + G + tol;
                    
            end           
            
            
            % sample activation space
            %combine inequalities for direction and muscle activation bounds
            %inequalities for muscle activations [0 1] for 98 muscles
            nMuscles = size(Sm,2);
            A=[A_specU; A_specL; diag(ones(nMuscles,1)); -1*diag(ones(nMuscles,1))];
            b=[b_specU; b_specL; ones(nMuscles,1); zeros(nMuscles,1)];

            options.x0 = chebycenter(A,b);
            options.method = 'hitandrun';

            n=1000000;
            [X,~] = cprnd(n,A,b,options);

        end
        
        
        function plotActSpaceInAccelSpace(act_vecs, pp, nn, neckParams, options)
            % options: options.Sm, options.FAS, options.max_accel,
            % options.plot_axes

           % Parameters from neckParams structure
            J = neckParams.Params(pp).Jacobian;            
            R = neckParams.Params(pp).R;
            Fa = diag(neckParams.Params(pp).Factive);
            %Fp = Cond.Posture(pp).Fpassive;
            Fg = neckParams.Params(pp).Fgrav;
            %Fb = Base.Posture(pp).Fbushing;
            Minv = neckParams.Params(pp).Minv;
            
            if isfield(options,'Sm')
                Sm = options.Sm;
            else
                Sm = 1;
            end

            % plot acceleration resulting from constrained activation space
            for ii=1:size(act_vecs,1)
                a=act_vecs(ii,:)';
                a_t(ii,:) = J * Minv * (R * Fa * Sm * a + Fg);
            end

            scatter3(a_t(:,1),a_t(:,2),a_t(:,3),'k','.')
            xlabel('a_x:Anterior')
            ylabel('a_y:Superior')
            zlabel('a_z:Right')
            axis equal

            hold on

            % max acceleration in that direction from FAS optimization
            if isfield(options,'FAS')
                a_max = options.FAS.Posture(pp).accel(:,nn);
                scatter3(a_max(:,1),a_max(:,2),a_max(:,3),'r','o')
            elseif isfield(options,'max_accel')
                if isvector(options.max_accel)
                    a_max = options.max_accel;
                elseif ismatrix(options.max_accel)
                    a_max = options.max_accel(:,nn);
                end
                scatter3(a_max(:,1),a_max(:,2),a_max(:,3),'r','o')
            end    
            
            if isfield(options,'plot_axes')
                if options.bool_axes == 1
                    %plot axes
                    fig = gcf;
                    ax = gca;
                    axes = neckParams.Params(pp).axes;
                    X_GF = utility.transformMatrix(axes.R_Gfh, axes.p_Gfh);
                    plotTools.plot_axes(X_GF, 30, fig, ax);
                    axis equal
                    axis vis3d
                end
            end
            hold off

        end
        
        function [fig,ax] = plotHisto_2D(act_vecs, options)
            % try changing some inputs to options.mms, options.mm_names, options.nrows,
            % options.nbins and use defaults if they don't exist. 
            
            %muscle indices
            if isfield(options, 'mmI')
                mmI = options.mmI;
            else
                mmI = [1 2 3 4 5 6 7 8 9 10 11 12 13 14 15 16]; %default all 16 muscles
                disp('using default mms (muscle indices)')
            end

            % muscle names
            if isfield(options, 'mm_names')
                mm_names = options.mm_names;
            else
                mm_names = ["sup ext R","sup ext L","suboccipitals R","suboccipitals L", ...
                    "intermed ext R", "intermediate ext L", "multifidus R", "multifidus L", ...
                    "longus cap R", "longus cap L", "longus colli R", "longus colli L", ...
                    "hyoids R", "hyoids L", "SCM R", "SCM L"]; % default all 16 names
                disp('using default: all 16 mm_names')
            end

            if isfield(options, 'nbins')
                nbins = options.nbins;
            else
                nbins = 20; % default 
                disp('using default nbins = 20')
            end

            if isfield(options, 'nrows')
                nrows = options.nrows;
            else
                nrows = 4; % default
                disp('using default nrows = 4')
            end
            
            nplots = length(mmI);
            ncols = ceil(nplots/nrows); %rounds UP to next integer


            fig = figure;
            for ii = 1 : nplots
                ax(ii) = subplot(nrows,ncols,ii);
                histogram(ax(ii),act_vecs(:,mmI(ii)),nbins,'BinLimits',[0,1],'Normalization','probability')

                mm_string = mm_names(mmI(ii));
                mm_string = replace(mm_string,"_"," "); %so it doesn't subscript
                title(mm_string)   

                hold on
            end

            set(ax, 'YLim', [0, 1]);

            % title will include posture info and max acceleration activations will show in histogram if FAS and direction index nn are included    
            if isfield(options,'FAS') && isfield(options,'nn') && isfield(options,'pp')
                FAS = options.FAS;
                nn = options.nn;
                pp = options.pp;
                postures = FAS.postures;
                pp_str = plotTools.postureString(postures,pp);
                fig.Name = sprintf('%d percent max accel in Direction %d in %s',p*100,nn,pp_str);

                for ii = 1 : nplots
                    max_act = FAS.Posture(pp).mmAct(mmI(ii),nn);
                    histogram(ax(ii),max_act * ones(size(act_vecs,1),1),nbins,'BinLimits',[0,1],'Normalization','probability')
                end
            elseif isfield(options,'postures') && isfield(options,'pp')
                postures = options.postures;
                pp = options.pp;
                fig.Name = plotTools.postureString(postures,pp);

            end
                
        end
        
        function [fig,ax,h] = plotHisto16mm_3D(Act, pp, nn, FWS, options)
            % Act is a structure across the task intensity (length of 10 would give 10
            % bins in 10% increments). Act(i).X contains the muscle activation patterns
            % for that task intensity. Act(i).p containts the fraction of total
            % intensity of that data set. 

            % pp = posture_index;
            % nn = direction_index;
            % FAS is a structure that contains the following: 
            %           postures (indexed by pp)
            %           unitvecs (indexed by nn)
            %           accel    (indexed by pp,nn)

            % options can include fields: 
            %           mms (muscle indices), 
            %           mm_names (strings matching indices), 
            %           nrows (# of rows of subplots), 
            %           nbins (# of activation level bins on x axis)
            %           style: 'tile' or 'bar3' ('bar' or 'bar_flat' also work)


            %muscle indices
            if isfield(options, 'mmI')
                mmI = options.mmI;
            else
                mmI = [1 2 3 4 5 6 7 8 9 10 11 12 13 14 15 16 17 18 19 20 21 22 23 24]; %default all 16 muscles
                disp('using default mms (muscle indices)')
            end

            % muscle names
            if isfield(options, 'mm_names')
                mm_names = options.mm_names;
            else
                mm_names = ["Deep Low Ext R", "Deep Low Ext L", "Deep Low Flex R", " Deep Low Flex L",...
                    "Deep Mid Ext R", "Deep Mid Ext L", "Deep Mid Flex R", "Deep Mid Upper Flex L", ...
                    "Deep Upper Ext R", "Deep Upper Ext L", "Deep Upper Flex R", "Deep Upper Flex L", ...
                    "Intermed Low Ext R", "Intermed Low Ext L", "Intermed Low Flex R", "Intermed Low Flex L",...
                    "Intermed Upper Ext R", "Intermed Upper Ext L", "Intermed Upper Flex R", "Intermed Upper Flex L",...
                    "Global Ext R", "Global Ext L", "Global Flex R", "Global Flex L"];
                disp('using default: all 24 mm_names')
            end

            if isfield(options, 'nbins')
                nbins = options.nbins;
            else
                nbins = 20; % default 
                disp('using default nbins = 20')
            end

            if isfield(options, 'nrows')
                nrows = options.nrows;
            else
                nrows = 3; % default
                disp('using default nrows = 3')
            end


            if isfield(options, 'style')
                if strcmp(options.style,'tile')
                    chart_style = 'tile';
                elseif strcmp(options.style,'bar') ...
                        || strcmp(options.style,'bar3') ...
                        || strcmp(options.style,'bar_flat')
                    chart_style = 'bar3';
                end
            else
                chart_style = 'tile'; %default
            end

            %combined with pcolor
            nplots = length(mmI);
            ncols = ceil(nplots/nrows); %rounds UP to next integer

            %name the figure
            if isfield(FWS,'postures')
                postures = FWS.postures;
                pp_str = plotTools.postureString(postures,pp);
                fig_title = sprintf('Direction %d in %s',nn,pp_str);
                fig = figure('Name',fig_title);
                sgtitle(fig_title);
            else 
                fig_title = sprintf('Direction %d in Posture %d',nn,pp);
                fig = figure('Name',fig_title);
                sgtitle(fig_title);
            end

            %ntasklevels = length(Act);
            Xedges = (0 : 1/nbins : 1); %Activation
            Yedges = [0 11 21 31 41 51 61 71 81 91 101 105]; %Task Level

            for mm = 1 : nplots
                                
                nLevels = length(Act); 
                nSamples = size(Act(1).X,1);
                
                actX = [];
                for level = 1 : nLevels
                    p = Act(level).p*100;                        
                    actX = [actX; Act(level).X(:,mm) p*ones(nSamples,1)];
                end
                
                % add optimal solution for max wrench 
                max_act = FWS.Posture(pp).mmAct(mmI(mm),nn);

                actX = [actX; max_act*ones(nSamples,1) 104*ones(nSamples,1)];
                [Ntotal,Xedges,Yedges] = histcounts2(actX(:,1),actX(:,2),Xedges,Yedges,'Normalization','pdf');
                
                
                ax(mm) = subplot(nrows,ncols,mm);

                h(mm) = histogram2('XBinEdges', Xedges, ...
                    'YBinEdges', Yedges, ...
                    'BinCounts', Ntotal, ...
                    'DisplayStyle', chart_style,...
                    'ShowEmptyBins','off', ...
                    'FaceColor', 'flat');

                xlabel('Act')
                ylabel('% max')
                zlabel('% sol')

                if isfield(options,'style')
                    if strcmp(options.style,'bar_flat')
                        view(2)
                    end
                end
                
                %find the max value excluding the optimal solution 
                maxValue(mm) = max(max(h(mm).Values(:,1:nLevels)));
                
                mm_string = mm_names(mmI(mm));
                mm_string = replace(mm_string,"_"," "); %so it doesn't subscript
                title(mm_string)   

            end

            if strcmp(chart_style,'tile')  
                linkaxes(ax,'xy')
                maxV = max(maxValue);
                for mm = 1 : nplots
                    caxis(h(mm), 'manual')
                    caxis(h(mm), [0 maxV])
                end
            elseif strcmp(chart_style,'bar3')
                hlink = linkprop(ax, {'CameraPosition','CameraUpVector','XLim', 'YLim', 'ZLim'});
                setappdata(fig, 'StoreTheLink', hlink);
                maxV = max(maxValue);
                zlim(ax,[0 maxV+0.001]);
                rotate3d on
                for mm = 1 : nplots
                    %caxis(h(mm), 'manual')
                    caxis(ax(mm), [0 maxV])
                end
            end
        end
        
        function [fig,ax,h] = plotHisto_3D(Act, options)
            % Act is a structure across the task intensity (length of 10 would give 10
            % bins in 10% increments). Act(i).X contains the muscle activation patterns
            % for that task intensity. Act(i).p containts the fraction of total
            % intensity of that data set. 

            % pp = posture_index;
            % nn = direction_index;
            % FAS is a structure that contains the following: 
            %           postures (indexed by pp)
            %           unitvecs (indexed by nn)
            %           accel    (indexed by pp,nn)

            % options can include fields: 
            %           mms (muscle indices), 
            %           mm_names (strings matching indices), 
            %           nrows (# of rows of subplots), 
            %           nbins (# of activation level bins on x axis)
            %           style: 'tile' or 'bar3' ('bar' or 'bar_flat' also work)


            %muscle indices
            if isfield(options, 'mmI')
                mmI = options.mmI;
            else
                mmI = [1 2 3 4 5 6 7 8 9 10 11 12 13 14 15 16 17 18 19 20 21 22 23 24]; %default all 16 muscles
                disp('using default mms (muscle indices)')
            end

            % muscle names
            if isfield(options, 'mm_names')
                mm_names = options.mm_names;
            else
                mm_names = ["sup ext R","sup ext L","suboccipitals R","suboccipitals L", ...
                    "intermed ext R", "intermediate ext L", "multifidus R", "multifidus L", ...
                    "longus cap R", "longus cap L", "longus colli R", "longus colli L", ...
                    "hyoids R", "hyoids L", "SCM R", "SCM L"]; % default all 16 names
                disp('using default: all 16 mm_names')
            end

            if isfield(options, 'nbins')
                nbins = options.nbins;
            else
                nbins = 20; % default 
                disp('using default nbins = 20')
            end

            if isfield(options, 'nrows')
                nrows = options.nrows;
            else
                nrows = 4; % default
                disp('using default nrows = 3')
            end


            if isfield(options, 'style')
                if strcmp(options.style,'tile')
                    chart_style = 'tile';
                elseif strcmp(options.style,'bar') ...
                        || strcmp(options.style,'bar3') ...
                        || strcmp(options.style,'bar_flat')
                    chart_style = 'bar3';
                end
            else
                chart_style = 'bar3'; %default
            end            
            
            %combined with pcolor
            nplots = length(mmI);
            ncols = ceil(nplots/nrows); %rounds UP to next integer

            %name the figure
%           sgtitle(fig_title);
            if isfield(options,'postures') && isfield(options,'pp')
                postures = options.postures;
                pp = options.pp;
                pp_str = plotTools.postureString(postures,pp);
                fig = figure('Name',pp_str);
            else
                fig = figure();
            end

            if isfield(options, 'task_levels')
                task_levels = options.task_levels;                
                nTaskLevels = length(task_levels);
                maxTask = task_levels(nTaskLevels);
                Yedges = [task_levels(1) : maxTask/nTaskLevels : maxTask];
            elseif isfield(options, 'nYbins') && isfield(Act,'p')
                nTaskLevels = length(Act);
                Yedges = [0 : Act(nTaskLevels).p/options.nYbins : Act(nTaskLevels).p]; 
            elseif isfield(options, 'nYbins') && isfield(Act,'head_mass')
                nTaskLevels = length(Act);
                Yedges = [0 : Act(nTaskLevels).p/options.nYbins : Act(nTaskLevels).p]; 
            else
                Yedges = [0 5:10:95 100 101]; %Task Level
            end
            %ntasklevels = length(Act);
            Xedges = (0 : 1/nbins : 1); %Activation

            for ii = 1 : nplots
                actX = [];

                for jj = 1 : length(Act)
                    actX = [actX; Act(jj).X(:,mmI(ii)) Act(jj).head_mass*ones(size(Act(jj).X,1),1)];    
                end
                %max_act = FAS.Posture(pp).mmAct(mmI(ii),nn);
                %DEBUG
                nLevels = length(Act);
                bins = [nbins,nLevels]; 
                [N,Xedges,Yedges] = histcounts2(actX(:,1),actX(:,2),bins,...
                    'XBinLimits', [0 1]);
                %'Normalization','pdf');
                %END DEBUG
                
                %[N,Xedges,Yedges] = histcounts2(actX(:,1),actX(:,2),Xedges,Yedges);

                %[Nmax, ~,~] = histcounts2(max_act, 101, Xedges, Yedges);

                %Ntotal = N + Nmax;

                ax(ii) = subplot(nrows,ncols,ii);

                h(ii) = histogram2('XBinEdges', Xedges, ...
                    'YBinEdges', Yedges, ...
                    'BinCounts', N, ...         %had been Ntotal
                    'DisplayStyle', chart_style,...
                    'ShowEmptyBins','off', ...
                    'FaceColor', 'flat');

                xlabel('Act')
                
                if isfield(options,'intensity_label')
                    ylabel(options.intensity_label);
                else
                    ylabel('intensity');
                end
                zlabel('# sol')

                if isfield(options,'style')
                    if strcmp(options.style,'bar_flat')
                        view(2)
                    end
                end

                maxValue(ii) = max(max(h(ii).Values));

                mm_string = mm_names(mmI(ii));
                mm_string = replace(mm_string,"_"," "); %so it doesn't subscript
                title(mm_string)   

            end

            if strcmp(chart_style,'tile')  
                linkaxes(ax,'xy')
                maxV = max(maxValue);
                for ii = 1 : nplots
                    caxis(h(ii), 'manual')
                    caxis(h(ii), [0 maxV])
                end
            elseif strcmp(chart_style,'bar3')
                hlink = linkprop(ax, {'CameraPosition','CameraUpVector','XLim', 'YLim', 'ZLim'});
                setappdata(fig, 'StoreTheLink', hlink);
                rotate3d on
            end
        end
        
        
        function [c,r] = chebycenter(A,b)
            % % from author of cprnd.m 
            %CHEBYCENTER Compute Chebyshev center of polytope Ax <= b.
            %  The Chebyshev center of a polytope is the center of the largest
            %  hypersphere enclosed by the polytope. 
            %  Requires optimization toolbox.

            [n,p] = size(A);
            an = sqrt(sum(A.^2,2));
            A1 = zeros(n,p+1);
            A1(:,1:p) = A;
            A1(:,p+1) = an;
            f = zeros(p+1,1);
            f(p+1) = -1;

            options = optimset;
            options = optimset(options,'Display', 'off');
            c = linprog(f,A1,b,[],[],[],[],[],options);
            r = c(p+1);
            c = c(1:p);
        end
        
        
        
        %input matrix of activation sets, where each row is a single
        %activation set. 
        %to get weighted cost (metabolic sense), multiply Fmax*act first. 
        function cost = computeCost(act,option)
                    
            switch option
               case '1norm'
                   p = 1;
               case '2norm'
                   p = 2;
               case 'maxnorm'
                   p = Inf;
               otherwise
                   p = 2;
                   warning('Invalid option input, taking 2-norm')
            end
            
            cost = vecnorm(act, p, 2);
           
        end
        
        
    end
end