function [Upot,Grad] = potentialPoint(map, goal, catt, crep, Q, loc)
% inputs:
%   map - map nx3 in the format xi,yi,ri, first line arena, others - obstacles
%   goal - [xgoal; ygoal]
%   catt - tunning parameter
%   crep - tunning parameter
%   Q    - tunning parameter
%   loc  - [x,y] location to check potential and gradient
% outputs:
%   Upot - total potential at point loc
%   Grad - gradient of steepest descent
%
%   Cornell University
%   MAE 5180: Autonomous Mobile Robots
%   HW #7
%   Scher, Guy

    % is it outside of the boundaries
    if( dist_func(map(1, 1:2)', loc) > map(1,3))
        Upot = NaN; Grad = [NaN; NaN];
        return
    end

    d = dist_func(goal, loc);
    % quadratic attraction
	Uatt = 0.5 * catt * d^2;
    Grad = catt * (loc-goal);
    
    Urep = 0;
    for i=2:size(map,1)
        di = dist_func(map(i, 1:2)', loc) - map(i, 3); % closest point is on the circumfrance
        e_vec = loc - map(i, 1:2)'; 
        e_vec = e_vec / norm(e_vec); % it's the same vector as (q-q*)/d(q,q*) for spheres
        if(di < 0)
            % this is inside the obstacles
            Urep = Urep+1000; 
            Grad = 0 * e_vec; %Grad - 0 * e_vec;
        elseif(di <= Q)
            if(di<=1e-3), di=1e-3; end % division by zero protection
        	Urep = Urep + 0.5 * crep * ( 1/di - 1/Q )^2;
            Grad = Grad + crep * (1/Q - 1/di) * 1/di^2 * e_vec;
        end
    end
    % total pot
    Upot = Urep + Uatt;
end

