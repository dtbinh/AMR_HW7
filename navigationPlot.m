function navigationPlot(mapFile, goal, k, lambda, locs, start, dim)
% inputs:
%   mapFile - map nx3 (matrix of file name) in the format xi,yi,ri, first line arena, others - obstacles
%   goal - [xgoal; ygoal]
%   k - tunning parameter
%   lambda - tunning parameter
%   locs  - nx[x,y]  location to check potential and gradient
%   start - [x;y] of start position (for plotting trajectory)
%   dim  - [Ny Nx] for converting locs back to meshgrid format
% outputs:
%   plot of navigation function and contours
%
%   Cornell University
%   MAE 5180: Autonomous Mobile Robots
%   HW #7
%   Scher, Guy

    if(ischar(mapFile))
        map = importdata(mapFile);
    else
        map = mapFile;
    end
    
    figure('Name','NavigationPlot'); 
    if(~isempty(start)), subplot(211); end
%     j=1;
%     for jj=linspace(.1,1000,6)
%         lambda=jj;
%         subplot(230+j);
    N = zeros(size(locs,1), 1);
    
    for i = 1:size(locs,1)
        Nav = spherePoint(map, goal, k, lambda, locs(i,:)');
        N(i) = Nav;
    end
    if(~isempty(start))
        [minS, minI] = min(sqrt( (locs(:,1)-start(1)).^2 + (locs(:,2)-start(2)).^2));
    end
    
    XX = reshape(locs(:,1), dim(2), dim(1));
    YY = reshape(locs(:,2), dim(2), dim(1));
    V  = reshape(N, dim(2), dim(1));
    
    surf(XX, YY, V,'EdgeColor','none');
    hold all; 
    plot3(goal(1), goal(2), 0, 'x', 'MarkerSize', 14, 'LineWidth',4, 'Color', [0 0.5 0]);
    if(~isempty(start))
        plot3(start(1), start(2), N(minI), 'ro', 'MarkerSize', 8, 'LineWidth',4);
    end
    zlim([0 1]); title(['Navigation function k=' num2str(k)]); xlabel('X [m]'); ylabel('Y [m]'); zlabel('Nav'); grid on; 
    
    if(~isempty(start))
        subplot(212); % figure(5);
    %     subplot(230+j);
        contour(XX, YY, V);
        hold all
        plot(goal(1), goal(2), 'x', 'MarkerSize', 14, 'LineWidth',4, 'Color', [0 0.5 0]);
        if(~isempty(start))
            plot(start(1), start(2), 'ro', 'MarkerSize', 8, 'LineWidth',4);
        end
        title(['Navigation function - Contours & Gradients k=' num2str(k)]); xlabel('X [m]'); ylabel('Y [m]'); grid on;
        [px,py] = gradient(V);

        px = reshape(px, dim(2)*dim(1),1); 
        py = reshape(py, dim(2)*dim(1),1);
        grad_norm = sqrt(px.^2+py.^2);
        px = px./grad_norm ; py = py./grad_norm ;
        px(isnan(px)) = 0; py(isnan(py)) = 0;
    %     G(:,1:2) = G(:,1:2) ./ sqrt(G(:,1).^2+G(:,2).^2);
        ii = 1:5:length(px);
        quiver(locs(ii,1), locs(ii,2), px(ii), py(ii),'Color',[.6 .6 .6]); axis equal;

        if(~isempty(start))
            curr = [start' N(minI)];
            alfa=0.1;
            cntr = 1000;

            while(cntr>=0 && dist_func(curr(end,1:2), goal') > 0.5)
                cntr = cntr - 1;
                [~, minI] = min(sqrt( (locs(:,1)-curr(end,1)).^2 + (locs(:,2)-curr(end,2)).^2));
                tmp = curr(end,1:2) - alfa*[px(minI) py(minI)];
                curr(end+1,:) = [tmp N(minI)];
            end
            plot(curr(:,1), curr(:,2), 'LineWidth', 3);
            subplot(211); % figure(4);
            plot3(curr(:,1), curr(:,2), curr(:,3), 'LineWidth', 3);
        end
    %     j=j+1;
    %     end
        px = reshape(px, dim(2),dim(1)); 
        py = reshape(py, dim(2),dim(1));
        save('navGrad.mat', 'px', 'py', 'XX', 'YY');
    else
        [px,py] = gradient(V);

        px = reshape(px, dim(2)*dim(1),1); 
        py = reshape(py, dim(2)*dim(1),1);
        grad_norm = sqrt(px.^2+py.^2);
        px = px./grad_norm ; py = py./grad_norm ;
        px(isnan(px)) = 0; py(isnan(py)) = 0;
        px = reshape(px, dim(2),dim(1)); 
        py = reshape(py, dim(2),dim(1));
        save('navGrad.mat', 'px', 'py', 'XX', 'YY');
    end
end

