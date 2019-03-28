function[dataStore] = navigationPlanner(CreatePort,DepthPort,TagPort,tagNum,maxTime)
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

prerec = importdata('navGrad2.mat');
px = prerec.px; py = prerec.py; XX = prerec.XX; YY = prerec.YY; 
goal = [0; 0];

map = [0, 0, 6 ; ...
       -3, -2, sqrt(2)*1.01; ...
       0, 3, sqrt(2)*1.01; ...
       2.5, -2.5, sqrt(1/2)*1.05 ];

alfa = 1;
robot_radius = 0.2; %[m]
e = robot_radius/2;
maxWheelV = 0.5; %[m/s]
wheel2Center = 0.13; %[m]
closeEnough = 0.1;

tic
while toc<maxTime    
    % Read and Store Sensore Data
    [noRobotCount,dataStore]=readStoreSensorData(CreatePort,DepthPort,TagPort,tagNum,noRobotCount,dataStore);
    pose  = dataStore.truthPose(end, 2:4);
    
    if( dist_func(pose(1:2)', goal) <= closeEnough )
        Vx=0; Vy=0;
    else
        Vx = -alfa*interp2(XX,YY,px,pose(1),pose(2));
        Vy = -alfa*interp2(XX,YY,py,pose(1),pose(2));
    end
    [cmdV, cmdW] = feedbackLin(Vx, Vy, pose(3), e);
    
    [cmdV, cmdW] = limitCmds(cmdV, cmdW, maxWheelV, wheel2Center);
    % if overhead localization loses the robot for too long, stop it
    if noRobotCount >= 3
        SetFwdVelAngVelCreate(CreatePort, 0, 0);
    else
        SetFwdVelAngVelCreate(CreatePort, cmdV, cmdW );
    end
    
    pause(0.1);
end

