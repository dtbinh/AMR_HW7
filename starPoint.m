function nav = starPoint(map, goal, k, lambda, loc)
% inputs:
%   map - map nx3 in the format xi,yi,ri, first line arena, others - obstacles
%   goal - [xgoal; ygoal]
%   k - tunning parameter
%   lambda - tunning parameter
%   loc  - [x,y]  location to check nav function
% outputs:
%   navigation function
%
%   Cornell University
%   MAE 5180: Autonomous Mobile Robots
%   HW #7
%   Scher, Guy

    Nobs = (size(map,1)-4)/4;
    betas = zeros(Nobs+1,1);
    v = zeros(Nobs+1,1);
    T = zeros(Nobs+1,2);
    b_bar=ones(Nobs+1,1);
    S = zeros(Nobs+1,1);
    sphere_map = zeros(Nobs+1,3);
    
    gamma = dist_func(loc, goal)^(2);%*k in the original phd there was no k
    
    % The Arena: the star center location
    q0 = [mean(map(1:4, 1)); mean(map(1:4, 2))];
    % radius which is bigger then the star arena
    r0 = sqrt((abs(map(1, 1)-map(3, 1))/2)^2 + ...
              (abs(map(1, 2)-map(2, 2))/2)^2);
    sphere_map(1,:) = [q0' r0]; % construct the sphere map (arena)
    betas(1) = -beta_func( loc, q0, r0, map(1:4, :) );
    v(1) = (1-betas(1)) * r0./dist_func(loc,q0); %sqrt
    v(1) = max(v(1), 1);
    T(1,:) = (v(1)*(loc-q0) + q0)';
    % calculate beta for each obstacle
    for i=1:Nobs
        % Each obstacle: the star center location
        qi = [mean(map(4*i+1:4*i+4, 1)); mean(map(4*i+1:4*i+4, 2))];
        % radius which is smaller then the star
        ri = min( abs(map(4*i+1, 2)-map(4*i+1, 4)), ...
                  abs(map(4*i+2, 1)-map(4*i+2, 3)) ) / 2; 

        betas(i+1) = beta_func( loc, qi, ri, map(4*i+1:4*i+4, :) );
        v(i+1) = (1+betas(i+1)) * ri./dist_func(loc,qi); %sqrt
        v(i+1) = min(v(i+1), 1);
        T(i+1,:) = (v(i+1)*(loc-qi) + qi)';
        sphere_map(i+1,:) = [qi' ri]; % construct the sphere map (obstacle i)
    end
    % calc beta_bar and the switch function
    for i=1:Nobs+1
        for j=1:Nobs+1
            if(i~=j)
                b_bar(i) = b_bar(i)*betas(j);
            end
        end
        S(i) = (gamma*b_bar(i))/(gamma*b_bar(i)+lambda*betas(i));
    end
    Sgoal = 1 - sum(S);
    h = Sgoal*(goal) + sum( S.*T )';
%     for debugging
%     figure(1); 
%     circle(sphere_map(1,1:2), sphere_map(1,3), 1000, 'k-');
%     for i=2:size(sphere_map,1)   
%         circle(sphere_map(i,1:2), sphere_map(i,3), 1000, 'r-');
%     end
%     plot(loc(1), loc(2), 'd', 'MarkerSize', 10);
%     plot(h(1), h(2), '*', 'MarkerSize', 10);
%     disp(['betas=[' num2str(betas(1)) ', ' num2str(betas(2)) ', ' num2str(betas(3)) ...
%         ', ' num2str(betas(4)) ']']);
%     disp(['S=[' num2str(S(1)) ', ' num2str(S(2)) ', ' num2str(S(3)) ...
%         ', ' num2str(S(4)) ']']);
    % now we're in sphere world
    [in,~] = inpolygon(loc(1),loc(2), ...
                        [map(1:4, 1); map(1, 1)], ...
                        [map(1:4, 2); map(1, 2)]);

    if(~in)
        nav = NaN; % this is not in the map so discard it anyway
    elseif( any(betas<0))
        % help it look nice inside obstacles cause it doesn't seem nice
        % there
        nav = 1;
    else
        nav = spherePoint(sphere_map, goal, k, lambda, h);
    end
end
% helper function to calculate the betas
function beta = beta_func( loc, qi, ri, obstacle_map )
    beta=[];
    for m=1:4
        [isect,xs,ys,~]= intersectPoint(loc(1),loc(2),qi(1),qi(2), ...
            obstacle_map(m, 1),obstacle_map(m, 2),obstacle_map(m, 3),obstacle_map(m, 4));

        if isect
            % we're outside or on the line
            beta = dist_func(loc, qi)^2 - dist_func([xs;ys], qi)^2;
%             beta = 0.5*dist_func(loc, qi)^2 ;%- ri^2;
        end
    end
    if isempty(beta)
        % meaning we're inside, but we're discarding this data anyway
        beta_x = min(min( abs(loc(1)-obstacle_map(:,1)) , abs(loc(1)-obstacle_map(:,3))));
        beta_y = min(min( abs(loc(2)-obstacle_map(:,2)) , abs(loc(2)-obstacle_map(:,4))));
        beta = -min(beta_x, beta_y)^2;
    end
end