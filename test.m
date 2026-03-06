%% Set up
clear; clc;

startPose = Pose(0, 0, 0, 0, 0, 0);
drive_bot = Drive(startPose);

%% MOVE
% from front plate of robot, (1st carriage linear motion, 2nd, 3rd, 1st
% tube rotational motion, 2nd, 3rd)
% drive_bot.travel_for(0,0,0,180,180,0)
drive_bot.travel_to(54,54,0,-43,-65,0)

