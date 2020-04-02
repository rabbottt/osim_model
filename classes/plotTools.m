classdef plotTools
    
    properties (Constant)
        %for plot_vectors
        vec_color = 'black';
        vec_line_width = 1.5;
        vec_show_arrowhead = 'off';
        vec_marker = 'none';
        
        %for plot_axes
        axis_length = 0.15;
        gnd_marker = 'diamond'; %to hide, change to 'none'
        gnd_marker_size = 15;
        axis_color = ['r', 'b', 'g'];
        grav_color = 'y'; %to hide, change to 'none'
        axis_lineStyle = '-.';
        grav_lineStyle = '--';        
        axes_line_width = 1;
        grav_line_width = 1;
        fh_marker = 'o';
        fh_marker_size = 15;
        fh_marker_color = 'b';
        
        %general
        x_label = 'X: Anterior';
        y_label = 'Y: Superior';
        z_label = 'Z: Right';

    end
    
    methods (Static)
        function [figObj, axObj] = setupNewFig(axes_layout)
            figObj = figure();
            if isscalar(axes_layout)
                for aa = 1 : axes_layout
                    axObj(aa) = subplot(axes_layout, 1, aa);
                    xlabel(plotTools.x_label)
                    ylabel(plotTools.y_label)
                    zlabel(plotTools.z_label)
%                     axObj(aa).DataAspectRatioMode = 'manual';
%                     axObj(aa).DataAspectRatio = [1 1 1];
%                     axObj(aa).PlotBoxAspectRatioMode = 'manual';
%                     axObj(aa).PlotBoxAspectRatio = [1 1 1];
                    axis vis3d
                end
                
            elseif ismatrix(axes_layout)
                p = 0;
                for aa = 1 : axes_layout(1)                   
                    for bb = 1 : axes_layout(2)
                        p = p + 1;
                        axObj(aa,bb) = subplot(axes_layout(1), axes_layout(2), p);
                        xlabel(plotTools.x_label)
                        ylabel(plotTools.y_label)
                        zlabel(plotTools.z_label)
                        
%                         axObj(aa,bb).DataAspectRatioMode = 'manual';
%                         axObj(aa,bb).DataAspectRatio = [1 1 1];
%                         axObj(aa,bb).PlotBoxAspectRatioMode = 'manual';
%                         axObj(aa,bb).PlotBoxAspectRatio = [1 1 1];
                        axis vis3d %new
                    end
                end
            end
            Link = linkprop(axObj(:),{'CameraUpVector', 'CameraPosition', 'CameraTarget', 'XLim', 'YLim', 'ZLim'});
            setappdata(figObj, 'StoreTheLink', Link);
            
            camup([0 1 0]);
%           axObj(1).CameraUpVector = [0, 1, 0]; %y is up direction
        end        
      
               
        function figObj = plot_vectors(x, y, z, origin, figObj, axObj)
            
            %check that length of vectors x y and z are the same
            if length(x) ~= length(y) || length(x) ~= length(z)
                error('length of x, y, and z must be equal.\n')         
            end
                        
            if length(origin) ~= 3
                error('origin input must be a 3x1 or 1x3 vector (x, y, z)')
            end
            
            %create origin vectors for plotting
            x0 = origin(1) * ones(length(x),1);
            y0 = origin(2) * ones(length(x),1);
            z0 = origin(3) * ones(length(x),1);

            %offset input vectors by origin
            px = x + x0;
            py = y + y0;
            pz = z + z0;
            
            % plot on the input fig and axes
            figure(figObj);
            axes(axObj);
            
            % Plot figure
            hold on;
            for ii = 1 : length(x)
                plot3([x0(ii);px(ii)],[y0(ii);py(ii)],[z0(ii);pz(ii)], ...
                  'Color', plotTools.vec_color, ...
                  'LineWidth', plotTools.vec_line_width, ...
                  'Marker', plotTools.vec_marker);
            end
            hold off;

        end
        
        function figObj = plotVectorMap(unit_vectors, intensity_vector, origin, figObj, axObj)           
            
            % make input fig and axes current 
            figure(figObj);
            axes(axObj);
            
            %get # of vectors
            nVecs = size(unit_vectors,1);
            
            %create origin vectors for plotting
            x0 = origin(1) * ones(nVecs,1);
            y0 = origin(2) * ones(nVecs,1);
            z0 = origin(3) * ones(nVecs,1);

           
            DT = delaunayTriangulation(unit_vectors);
            K = convexHull(DT);
            
            %offset input vectors by origin
            x = DT.Points(:,1) + x0;
            y = DT.Points(:,2) + y0;
            z = DT.Points(:,3) + z0;

            %properties from Patch properties
            trisurf(K, x, y, z, intensity_vector, 'FaceColor', 'interp');
            colorbar;
            shading interp;            

        end
        
        %should this be transformed in any way to match the posture? 
        function figObj = plotConvexHull(vectors, origin, figObj, axObj)           

            figure(figObj);
            axes(axObj);
            
            DT = delaunayTriangulation(vectors);
            K = convexHull(DT);
            
            %get # of vectors
            nVecs = length(DT.Points(:,1));
            
            %create origin vectors for plotting
            x0 = origin(1) * ones(nVecs,1);
            y0 = origin(2) * ones(nVecs,1);
            z0 = origin(3) * ones(nVecs,1);
            
            %offset points by origin
            x = DT.Points(:,1) + x0;
            y = DT.Points(:,2) + y0;
            z = DT.Points(:,3) + z0;

            %properties from Patch properties
            trisurf(K, x, y, z, ...
                'FaceColor', 'interp', ...
                'FaceAlpha', 0.7, ...
                'EdgeColor', 'flat', ...
                'LineStyle', '-');
%             colorbar;
%             shading interp;            

        end
        
       
        % to create X_GF, use: 
        % X_GF = utility.transformMatrix(axes.R_Gfh, axes.p_Gfh);
        function figObj = plot_axes(X_GF, axis_length, figObj, axObj)
            
            %make our figure the current figure
            figure(figObj);
            axes(axObj);
            hold on
            grid on
            
            %axes structure, all vectors expressed in ground frame
            % using homogenous coordinates 
            x_axis = [axis_length * 1, 0, 0, 1]';
            y_axis = [0, axis_length * 1, 0, 1]';
            z_axis = [0, 0, axis_length * 1, 1]';

            % forehead location expressed in ground frame (4x1)
            pFH = [X_GF(1,4), X_GF(2,4), X_GF(3,4), 1]; 
            
            % express axis of local body frame in ground frame (4x1)
            pFH_xa = X_GF * x_axis; 
            pFH_ya = X_GF * y_axis;
            pFH_za = X_GF * z_axis;

            %gravity vector, along negative y-axis (in ground frame)
            pGrav = [0, axis_length * -1, 0];

            
            %make marker at ground origin
            gnd = plot3(0, 0, 0, ...
                'MarkerSize', plotTools.gnd_marker_size,...
                'MarkerEdgeColor', 'black',...
                'Marker', plotTools.gnd_marker, ...
                'LineStyle', 'none');         
            
            %make marker at forehead origin
            fh = plot3(pFH(1), pFH(2), pFH(3), ...
                'MarkerSize', plotTools.fh_marker_size,...
                'MarkerEdgeColor', plotTools.fh_marker_color,...
                'Marker', plotTools.fh_marker, ...
                'LineStyle', 'none');   
            
            
            %gravity Vector drawn at origin
            grav = plot3([0,pGrav(1)], [0,pGrav(2)], [0, pGrav(3)], ...
                'Color', plotTools.grav_color, ...
                'LineWidth', plotTools.grav_line_width, ...
                'LineStyle', plotTools.grav_lineStyle);    
            
            %plot x-axis
            xaxis = plot3([pFH(1),pFH_xa(1)], [pFH(2),pFH_xa(2)], [pFH(3), pFH_xa(3)],...
                'Color', plotTools.axis_color(1), ...
                'LineWidth', plotTools.axes_line_width, ...
                'LineStyle', plotTools.axis_lineStyle);

            %plot y-axis
            yaxis = plot3([pFH(1),pFH_ya(1)], [pFH(2),pFH_ya(2)], [pFH(3), pFH_ya(3)], ...
                'Color', plotTools.axis_color(2), ...
                'LineWidth', plotTools.axes_line_width, ...
                'LineStyle', plotTools.axis_lineStyle);

            %plot z-axis 
            zaxis = plot3([pFH(1),pFH_za(1)], [pFH(2),pFH_za(2)], [pFH(3), pFH_za(3)], ...
                'Color', plotTools.axis_color(3), ...
                'LineWidth', plotTools.axes_line_width, ...
                'LineStyle', plotTools.axis_lineStyle);

            lgd = legend([xaxis yaxis zaxis grav gnd], {"skull x-axis: anterior", "skull y-axis: superior", "skull z-axis: right", "gravity", "ground origin"});
            lgd.Visible='off';
                        
            grid on

        end
        
        
        function pp_str = postureString(postures, posture_index)
            p = postures(posture_index,:);
            if length(p) == 9
                pp_str = sprintf('Posture %d: [%d, %d, %d, %d, %d, %d, %d, %d, %d]', ...
                    posture_index, p(1), p(2), p(3), p(4), p(5), p(6), p(7), p(8), p(9));
            elseif length(p) == 6
                pp_str = sprintf('Posture %d: [%d, %d, %d, %d, %d, %d]', ...
                    posture_index, p(1), p(2), p(3), p(4), p(5), p(6));
            else
                pp_str = sprintf('Posture %d',posture_index);
                warning('length of posture does not match postureString function. Must be 6 or 9 coords');
            end
        end
        
        %This does not require that you set up a new figure first. It does
        %it itself
        function [fig, ax] = plotmmAct(FAS, pp, mm_indices)

            uv = FAS.unitvecs;
            num_axes = length(mm_indices);

            [fig, ax] = plotTools.setupNewFig(num_axes);
            
            
            pp_str = plotTools.postureString(FAS.postures, pp);
            fig.Name = sprintf('Muscle Activation Vector Map for %s',pp_str);

            for ii = 1 : length(mm_indices)

                mm = mm_indices(ii);
                mmActi = FAS.Posture(pp).mmAct(mm,:)';

                plotTools.plotVectorMap(uv, mmActi, fig, ax(ii));

                axes = FAS.ppParams(pp).axes;
                %rotate but don't translate
                p = [0, 0, 0];
                X_GF = utility.transformMatrix(axes.R_Gfh, p);

                plotTools.plot_axes(X_GF, 1.5, fig, ax(ii));


                mm_name = FAS.mmConstants.Name(mm);
                ax_title = sprintf('Plot %d: %s activation', ii, mm_name);
                title(ax(ii), ax_title);
            end
        end
        
        
        function projFAStoWall(FAS, pp, figObj, axObj, boundary_color)
            
            %if boundary_color was not provided, set to default blue 'b'
            if nargin < 5
                boundary_color = 'b';
            end
            
            laser = FAS.ppParams(pp).laser_location;

            accel = FAS.Posture(pp).accel';
            accel_wall = [accel(:,3), accel(:,2)]; % wall is y-z plane where (+)y is up

            %scale vecs to max 0.1 for plotting purposes
            scale_to = 0.5;
            accel_wall_s = utility.scaleVecs(accel_wall, scale_to);

            %location of laser point on the wall (wall is in y-z plane)            
            y = laser(2); %y-component of laser
            z = laser(3); %z-component of laser

            DT = delaunayTriangulation(accel_wall_s);
            [C,~] = convexHull(DT);
            pts = DT.Points + [z,y];

            figure(figObj);
            axes(axObj);
            hold on
            plot(z,y,'o','MarkerSize', 5, 'MarkerFaceColor', 'r')
            plot(pts(C,1),pts(C,2), boundary_color)
            xlabel('Z')
            ylabel('Y')

        end
        
    end
    
end