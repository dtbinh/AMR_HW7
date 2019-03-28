function [V, W] = feedbackLin(Vx, Vy, theta, e)
%   Cornell University
%   MAE 5180: Autonomous Mobile Robots
%   Scher, Guy
% case it's inertial
if(e == 0)
    % protect division by zero
    e = eps;
end
out = [1 0; 0 1/e] * [cos(theta)  sin(theta); -sin(theta) cos(theta)]*[Vx; Vy];
% case it's in body-frame
% out = [1 0; 0 1/e] *[Vx; Vy];
V = out(1);
W = out(2);