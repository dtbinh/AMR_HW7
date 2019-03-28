close all
clear all
clc

%% import

data = importdata('hw7Sphere.txt');

%% plot
figure; hold all;
circle(data(1,1:2), data(1,3), 1000, 'k-');

for i=2:size(data,1)
   
    circle(data(i,1:2), data(i,3), 1000, 'r-');
    
end
title('Sphere world'); grid on; xlabel('X [m]'); ylabel('Y [m]'); legend('Boundary', 'Obstacle');

%% potential function
close all

N=256; M=256;%68;
goal = [30; 40];
start = [80; 55];

catt = .4; crep = 1000; Q = 2; % no local minimums
% catt = .01; crep = 100; Q = 10; % with local minimums

% [T,R] = meshgrid(linspace(0,2*pi,N),linspace(.1,data(1,3),M));
% X = data(1,1) + R.*cos(T);
% Y = data(1,2) + R.*sin(T);
[X,Y] = meshgrid(linspace(10,90,N),linspace(10,90,M));
XY = [X(:) Y(:)];

potentialPlot('hw7Sphere.mat', goal, catt, crep, Q, XY, start, [N M]);


%% Navigation function
% close all
clc

N=256; M=256;
goal = [30; 40];
start = [80; 55];

k = 6; lambda = 50; % 
% k = 6; lambda = 100; % 
% k = 15; lambda = .1; % 
[X, Y] = meshgrid(linspace(min(data(:,1)-data(:,3)),max(data(:,1)+data(:,3)),N), ...
                  linspace(min(data(:,2)-data(:,3)),max(data(:,2)+data(:,3)),M));
XY = [X(:) Y(:)];

navigationPlot('hw7Sphere.mat', goal, k, lambda, XY, start, [N M]);

%% control
close all

% map = [1.991 -1.990 1.991 -3.050; ...
% 	 1.991 -3.050 3.009 -3.050; ...
% 	 3.009 -3.050 3.009 -1.990; ...
% 	 3.009 -1.990 1.991 -1.990; ...
% 	 -0.957 3.990 -0.957 1.990; ...
% 	 -0.957 1.990 1.009 1.990; ...
% 	 1.009 1.990 1.009 3.990; ...
% 	 1.009 3.990 -0.957 3.990; ...
% 	 -1.957 -0.990 -1.957 -3.010; ...
% 	 -1.957 -3.010 -3.991 -3.010; ...
% 	 -3.991 -3.010 -3.991 -0.990; ...
% 	-3.991 -0.990 -1.957 -0.990]; % hw7map
map = [-5	5	-5	-5
  -5	-5	5	-5
	 5	-5	5	5
	 5	5	-5	5
	 -3	-1	-3	-3
	 -3	-3	-.5	-3
	 -.5	-3	-.5	-1
	 -.5	-1	-3	-1
	 -2	3.5	-2	1.5
	 -2	1.5	2.5	1.5
	 2.5	1.5	2.5	3.5
	 2.5	3.5	-2	3.5
	 1.5	0	1.5	-2.5
	 1.5	-2.5	3.5	-2.5
	 3.5	-2.5	3.5	0
	 3.5	0	1.5	0]; % hw7map
 
% data = [0, 0, 6 ; ...
%         -3, -2, sqrt(2)*1.01; ...
%         0, 3, sqrt(2)*1.01; ...
%         2.5, -2.5, sqrt(1/2)*1.05 ];

    goal = [0;0]; start=[-4;4];
figure; hold on;
for j=1:length(map(:,1))
    plot([map(j,1) map(j,3)], [map(j,2) map(j,4)], 'LineWidth', 2, 'Color', 'm'); 
end
% circle(data(1,1:2), data(1,3), 1000, 'k-');
% for i=2:size(data,1)   
%     circle(data(i,1:2), data(i,3), 1000, 'r-');
% end
grid on; xlabel('X_I [m]'); ylabel('Y_I [m]'); axis equal; 

global dataStore
pose = dataStore.truthPose(:, 2:end);
plot(pose(:,1), pose(:,2), 'b-', 'LineWidth' ,2);
plot(goal(1), goal(2), 'x', 'MarkerSize', 14, 'LineWidth',1, 'Color', [0 0.5 0]);
plot(start(1), start(2), 'ro', 'MarkerSize', 8, 'LineWidth',1);
title('Simulation with Navigation function - Start=[-4;4;0^o]');
%%
N=256; M=256;
goal = [0; 0];
start = [-4; 4];

catt = 4; crep = 10; Q = 0.3; % no local minimums
% catt = .01; crep = 100; Q = 10; % with local minimums

[X,Y] = meshgrid(linspace(-5,5,N),linspace(-5,5,M));
% X = data(1,1) + R.*cos(T);
% Y = data(1,2) + R.*sin(T);
XY = [X(:) Y(:)];

potentialPlot(data, goal, catt, crep, Q, XY, start, [N M]);
%% Navigation function for create
% close all
clc

N=256; M=256;
goal = [0; 0];
start = [4; -4];
k = 6; lambda = 50; % 
% k = 6; lambda = 100; % 
data = [0, 0, 6 ; ...
        -3, -2, sqrt(2)*1.01; ...
        0, 3, sqrt(2)*1.01; ...
        2.5, -2.5, sqrt(1/2)*1.05 ];

[X, Y] = meshgrid(linspace(min(data(:,1)-data(:,3)),max(data(:,1)+data(:,3)),N), ...
                  linspace(min(data(:,2)-data(:,3)),max(data(:,2)+data(:,3)),M));
XY = [X(:) Y(:)];

navigationPlot(data, goal, k, lambda, XY, start, [N M]);


%% star-map
% close all
clc

map = [-5	5	-5	-5
  -5	-5	5	-5
	 5	-5	5	5
	 5	5	-5	5
	 -3	-1	-3	-3
	 -3	-3	-.5	-3
	 -.5	-3	-.5	-1
	 -.5	-1	-3	-1
	 -2	3.5	-2	1.5
	 -2	1.5	2.5	1.5
	 2.5	1.5	2.5	3.5
	 2.5	3.5	-2	3.5
	 1.5	0	1.5	-2.5
	 1.5	-2.5	3.5	-2.5
	 3.5	-2.5	3.5	0
	 3.5	0	1.5	0]; % hw7map

% data = [0, 0, 6 ; ...
%         -3, -2, sqrt(2)*1.01; ...
%         0, 3, sqrt(2)*1.01; ...
%         2.5, -2.5, sqrt(1/2)*1.05 ];

goal = [0; 0];
start = [-4; 4];
% k = 6; lambda = 500000; 
k = 12; lambda = 200000; 
% k = 12; lambda = 50000; 
N=256; M=256;

[X, Y] = meshgrid(linspace(-7,7,N), ...
                  linspace(-7,7,M));
XY = [X(:) Y(:)];

figure(1); hold on; axis equal;
for j=1:length(map(:,1))
    plot([map(j,1) map(j,3)], [map(j,2) map(j,4)], 'LineWidth', 2, 'Color', 'm'); 
end
% starPoint(map, goal, k, lambda, [0; 0]);
% lambda = 1000000;
% for kk=5:-0.1:0
%     disp(num2str(kk));
%     starPoint(map, goal, k, lambda, beta, [0; kk]);
%     pause
% end
% N=10; M=10;
% [X, Y] = meshgrid(linspace(-5,5,N), ...
%                   linspace(-5,5,M));
% XY = [X(:) Y(:)];
% for i = 1:size(XY,1)
%     for j = 1:size(Y,1)
%         starPoint(map, goal, k, lambda, XY(i,:)')
%     end
% end
navigationStarPlot(map, goal, k, lambda, XY, start, [N M])
% spherePoint(map, goal, k, lambda, [2;2]);
% circle(data(1,1:2), data(1,3), 1000, 'k-');
% for i=2:size(data,1)   
%     circle(data(i,1:2), data(i,3), 1000, 'r-');
% end
% grid on; xlabel('X_I [m]'); ylabel('Y_I [m]'); axis equal; 
% 
% global dataStore
% pose = dataStore.truthPose(:, 2:end);
% plot(pose(:,1), pose(:,2), 'b-', 'LineWidth' ,2);
% plot(goal(1), goal(2), 'x', 'MarkerSize', 14, 'LineWidth',1, 'Color', [0 0.5 0]);
% plot(start(1), start(2), 'ro', 'MarkerSize', 8, 'LineWidth',1);
% title('Simulation with Navigation function - Start=[-4;4;20^o]');

