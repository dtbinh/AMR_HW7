function[dataStore] = potentialPlanner(CreatePort,DepthPort,TagPort,tagNum,maxTime)
% freedriveProgram: example program to manually drive iRobot Create
% Reads data from sensors, sends robot commands, and saves a datalog.
% 
%   DATASTORE = freedriveProgram(CreatePort,DepthPort,TagPort,tagNum,maxTime)
% 
%   INPUTS
%       CreatePort  Create port object (get from running CreatePiInit)
%       DepthPort   Depth port object (get from running CreatePiInit)
%       TagPort     Tag port object (get from running CreatePiInit)
%       tagNum      robot number for overhead localization
%       maxTime     max time to run program (in seconds)
% 
%   OUTPUTS
%       dataStore   struct containing logged data
% 
%   Cornell University
%   MAE 5180: Autonomous Mobile Robots
%   HW #7
%   Scher, Guy

% Set unspecified inputs
defaultMaxTime = 500;
if nargin < 1
    disp('ERROR: TCP/IP port object not provided.');
    return;
elseif nargin < 2
    DepthPort = CreatePort;
    TagPort = CreatePort;
    tagNum = CreatePort;
    maxTime = defaultMaxTime;
elseif nargin < 3
    TagPort = CreatePort;
    tagNum = CreatePort;
    maxTime = defaultMaxTime;
elseif nargin < 4
    tagNum = CreatePort;
    maxTime = defaultMaxTime;
elseif nargin < 5
    maxTime = defaultMaxTime;
end

% declare datastore as a global variable so it can be accessed from the
% workspace even if the program is stopped
global dataStore;

% initialize datalog struct (customize according to needs)
dataStore = struct('truthPose', [],...
                   'odometry', [], ...
                   'rsdepth', [], ...
                   'bump',[]);
               
noRobotCount = 0;

goal = [0; 0];

% catt = 40; crep = 10; Q = 0.3; % no local minimums
catt = 2; crep = 10; Q = 1; % no local minimums
% catt = .01; crep = 100; Q = 10; % with local minimums

map = [0, 0, 6 ; ...
       -3, -2, sqrt(2)*1.01; ...
       0, 3, sqrt(2)*1.01; ...
       2.5, -2.5, sqrt(1/2)*1.05 ];

alfa = 1;
robot_radius = 0.2; %[m]
e = robot_radius/2;
maxWheelV = 0.4999; %[m/s]
wheel2Center = 0.13; %[m]
closeEnough = 0.1;

tic
while toc<maxTime    
    % Read and Store Sensore Data
    [noRobotCount,dataStore]=readStoreSensorData(CreatePort,DepthPort,TagPort,tagNum,noRobotCount,dataStore);
    pose  = dataStore.truthPose(end, 2:4);
    
    [~, Grad] = potentialPoint(map, goal, catt, crep, Q, pose(end, 1:2)');
    Grad = Grad / norm(Grad);
    V = -alfa * Grad;
    if( dist_func(pose(1:2)', goal) <= closeEnough )
        V = [0;0];
    end
    [cmdV, cmdW] = feedbackLin(V(1), V(2), pose(3), e);
    
    [cmdV, cmdW] = limitCmds(cmdV, cmdW, maxWheelV, wheel2Center);
%     disp(['t=' num2str(dataStore.truthPose(end, 1)) ' cmdV=' num2str(cmdV) ' cmdW=' num2str(cmdW)])
    
    % if overhead localization loses the robot for too long, stop it
    if noRobotCount >= 3
        SetFwdVelAngVelCreate(CreatePort, 0, 0);
    else
        SetFwdVelAngVelCreate(CreatePort, cmdV, cmdW );
    end
    
    pause(0.1);
end

