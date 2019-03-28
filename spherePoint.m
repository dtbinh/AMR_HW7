function nav = spherePoint(map, goal, k, lambda, loc)
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
    % just like in the books
    gamma = dist_func(loc, goal)^(2*k);
    betas = zeros(1,size(map,1));
    
    betas(1) = -dist_func(loc, map(1,1:2)')^2 + map(1,3)^2; %-d^2+r^2
    for i=2:size(map,1)
        betas(i) = dist_func(loc, map(i,1:2)')^2 - map(i,3)^2; %d^2-r^2
    end
    beta = prod(betas);

    if(betas(1) < 0 )
        nav = NaN;
    elseif( beta < 0 )
        nav = 1;
    else
        nav = (gamma/(lambda*beta+gamma))^(1/k);
    end
end