%% Set up
clear; clc;

startPose = Pose(0, 0, 0, 0, 0, 0);
drive_bot = Drive(startPose);

%% MOVE
% from front plate of robot, (1st carriage linear motion, 2nd, 3rd, 1st
% tube rotational motion, 2nd, 3rd)
drive_bot.travel_for(10,0,0,0,0,0)

