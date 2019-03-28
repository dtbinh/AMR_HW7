function[cmdV,cmdW] = limitCmds(fwdVel,angVel,maxV,wheel2Center)
% LIMITCMDS: scale forward and angular velocity commands to avoid
% saturating motors.
% 
%   [CMDV,CMDW] = LIMITCMDS(FWDVEL,ANGVEL,MAXV,WHEEL2CENTER) returns the 
%   forward and angular velocity commands to drive a differential-drive 
%   robot whose wheel motors saturate at +/- maxV.
% 
%   INPUTS
%       fwdVel      desired forward velocity (m/s)
%       angVel      desired angular velocity (rad/s)
%       maxV        maximum motor velocity (assumes symmetric saturation)
%       wheel2Center distance from the wheels to the center of the robot(in meters)
% 
%   OUTPUTS
%       cmdV        scaled forward velocity command (m/s)
%       cmdW        scaled angular velocity command (rad/s)
% 
% 
%   Cornell University
%   Autonomous Mobile Robots
%   Homework #1
%   Scher, Guy
ru_r = fwdVel + angVel*wheel2Center;
ru_l = fwdVel - angVel*wheel2Center;
ratio = max(abs(ru_r), abs(ru_l))/maxV;
if(ratio > 1)
    % scale to avoid sat.
    ru_r = ru_r / ratio;
    ru_l = ru_l / ratio;
end
% return scaled commands
cmdV = 1/2 *( ru_r + ru_l );
cmdW = 1/(2*wheel2Center) *( ru_r - ru_l );

if(abs(cmdW)<1e-6), cmdW=0; end %this is some bug in the simulator
