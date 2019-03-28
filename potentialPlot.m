function potentialPlot(mapFile, goal, catt, crep, Q, locs, start, dim)
% inputs:
%   mapFile - map nx3 (matrix of file name) in the format xi,yi,ri, first line arena, others - obstacles
%   goal - [xgoal; ygoal]
%   catt - tunning parameter
%   crep - tunning parameter
%   Q    - tunning parameter
%   locs  - nx[x,y]  location to check potential and gradient
%   start - [x;y] of start position (for plotting trajectory)
%   dim  - [Ny Nx] for converting locs back to meshgrid format
% outputs:
%   plot of potential and contours
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
    figure('Name','PotentialPlot'); 
    if(~isempty(start)), subplot(211); end
    Z = zeros(size(locs,1), 1);
    G = zeros(size(locs,1), 2);
    % get the actual values
    for i = 1:size(locs,1)
        [Upot, Grad] = potentialPoint(map, goal, catt, crep, Q, locs(i,:)');
        Z(i) = Upot;
        G(i,:) = Grad';
    end
    Z(Z>1000)=1000; % clip the too big numbers
    % plotting
    surf(reshape(locs(:,1), dim(2), dim(1)), reshape(locs(:,2), dim(2), dim(1)), reshape(Z, dim(2), dim(1)),'EdgeColor','none');
    hold all; 
    plot3(goal(1), goal(2), 0, 'x', 'MarkerSize', 8, 'LineWidth',4, 'Color', [0 0.5 0]);
    if(~isempty(start))
        [Upot_start, ~] = potentialPoint(map, goal, catt, crep, Q, start);
        plot3(start(1), start(2), Upot_start, 'ro', 'MarkerSize', 8, 'LineWidth',4);
    end
    zlim([0 1000]); title('Potential field'); xlabel('X [m]'); ylabel('Y [m]'); zlabel('U_p_o_t'); grid on; 
    
    if(~isempty(start))
        subplot(212); %     figure(3);
        contour(reshape(locs(:,1), dim(2), dim(1)), reshape(locs(:,2), dim(2), dim(1)), reshape(Z, dim(2), dim(1)));
        hold all
        plot(goal(1), goal(2), 'x', 'MarkerSize', 14, 'LineWidth',4, 'Color', [0 0.5 0]);
        if(~isempty(start))
            plot(start(1), start(2), 'ro', 'MarkerSize', 8, 'LineWidth',4);
        end
        title('Potential Field - Contours & Gradients'); xlabel('X [m]'); ylabel('Y [m]'); grid on;
        G(:,1:2) = G(:,1:2) ./ sqrt(G(:,1).^2+G(:,2).^2);

        ii = 1:5:length(G(:,1));
        quiver(locs(ii,1), locs(ii,2), G(ii,1), G(ii,2),'Color',[.6 .6 .6]); axis equal;
        % find trajectory
        if(~isempty(start))
            curr = [start' 0];
            alfa=0.1;
            cntr = 1000;
            % gradient descent
            while(cntr>=0 && dist_func(curr(end,1:2), goal') > 0.5)
                cntr = cntr - 1;
                [U, Grad] = potentialPoint(map, goal, catt, crep, Q, curr(end,1:2)');
                tmp = curr(end,1:2) - alfa*Grad'/norm(Grad);
                curr(end+1,:) = [tmp U];
            end
            plot(curr(:,1), curr(:,2), 'LineWidth', 3);
            subplot(211); % figure(2);
            plot3(curr(:,1), curr(:,2), curr(:,3), 'LineWidth', 3);
        end
    end
end

