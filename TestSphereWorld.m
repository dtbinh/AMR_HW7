function TestSphereWorld(map, goal, c_att, c_rep, Q, k, lambda, points)
% TESTSPHEREWORLD
% Test function for MAE 4180/5180 CS 3758, Homework 7. 
% Plots the potential field and navigation function.
% Will create two (2) figures, one containing the potential field and the other the navigation function.
%
%       TestSphereWorld(map, goal, c_att, c_rep, Q, k, lambda, points)
%
%       INPUTS:
%           map         map of the environment defined by circles.
%                       k x 3 matrix [x_center y_center radius]
%           goal        goal point
%                       1 x 2 array [x_goal y_goal]
%           c_att       scaling factor for the atractive force (potential fun).
%           c_rep       scaling factor for the repulsive force (potential fun).
%           Q           influence range of the obstacles (real number)
%           k           scaling factor for the atractive force (navigation fun).
%           lambda      scaling factor for the inverse repulsive force (navigation fun).
%           points      list of points to be evaluate
%                       n x 2 matrix [x, y]
%           
%                       
%       OUTPUTS:
%           none
%       Figures Created:
%           Figure 1    Potential field 
%           Figure 2    Navigation function 
%
% Autonomous Mobile Robots
% 

%~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
%       POTENTIAL FUNCTION
%~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
% reslove the ICD
N2 = size(points,1);
N  = floor( sqrt(N2) );

% [points_x, points_y] = meshgrid(points(:,1)',points(:,2)');
% new_points = [points_x(:) points_y(:)];
% STUDENTS: Call the function potentialPlot to generate the potential field
% plot in the specified points
potentialPlot(map, goal', c_att, c_rep, Q, points, [], [N N]);


%~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
%       NAVIGATION FUNCTION
%~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

% STUDENTS: Call the function navigationPlot to generate the navigation
% function plot in the specified points
navigationPlot(map, goal', k, lambda, points, [], [N N]);
%END
end
