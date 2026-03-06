%% Set up
clear; clc;
% Inverse Kinematics of a Continuum Robot
clear
close all
clc
% Create a robot with 3 links

OD = [1.8 1.6 1.4]; % link diameters [mm]

tube1 = Tube(3.046*10^-3, 3.3*10^-3, 1/8, 90*10^-3, 50*10^-3, 1935*10^6);
tube2 = Tube(2.386*10^-3, 2.64*10^-3, 1/11, 170*10^-3, 50*10^-3, 1935*10^6);
tube3 = Tube(1.726*10^-3, 1.98*10^-3, 1/29, 250*10^-3, 50*10^-3, 1935*10^6);

% TODO You will need to uncomment one of the below lines, depending if you
% are testing a 2-tube or 3-tube case
%tubes = [tube1, tube2];
tubes = [tube1, tube2, tube3];
n=2;
robot = Robot(tubes(1:n));

startPose = Pose(0, 0, 0, 0, 0, 0);
drive_bot = Drive(startPose);

%%
p=[-25;20;30].*10^-3;
x=p(1);
y=p(2);
z=p(3);
guess=[-10, 10, -45, -45];
Q=robot.calculate_ik([1 0 0 x; 0 1 0 y; 0 0 1 z],guess);
fprintf('goal pos: (%g,%g,%g)\n', x*10^3, y*10^3, z*10^3);
fprintf('q:(%.0f,%.0f,0,%.0f,%.0f,0)\n', Q(1),Q(2),Q(3),Q(4));
fprintf('actual pos: \n')
out=robot.fkin(Q);

%% MOVE
% from front plate of robot, (1st carriage linear motion, 2nd, 3rd, 1st
% tube rotational motion, 2nd, 3rd)
% drive_bot.travel_for(0,0,0,180,180,0)
drive_bot.travel_to(-61,-12,0,-128,-130,0)

%%
drive_bot.travel_to(0,0,0,0,0,0)
