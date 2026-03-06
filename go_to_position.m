%% Set up
clear; clc;

% startPose = Pose(0, 0, 0, 0, 0, 0);
% drive_bot = Drive(startPose);

%% INPUT
% Initial joint values (for 2 tube case)
q_initial = [20, 50, 45, -45];

% Goal position
goal_pos = [0.0093, -0.0309, 0.0691]';

% Reference q used in debugging; The q corresponding to the goal position
q_ref = [20, 50, 45, -45];
rho_ref = q_ref(1:length(q_ref)/2)/1000;
theta_ref = deg2rad(q_ref(1+length(q_ref)/2:end));


%% CALCULATIONS
tube1 = Tube(3.046*10^-3, 3.3*10^-3, 1/17, 90*10^-3, 50*10^-3, 1935*10^6);
tube2 = Tube(2.386*10^-3, 2.64*10^-3, 1/22, 170*10^-3, 50*10^-3, 1935*10^6);
tube3 = Tube(1.726*10^-3, 1.98*10^-3, 1/29, 250*10^-3, 50*10^-3, 1935*10^6);

% Uncomment one of the below lines, depending if you are testing a 2-tube or 3-tube case
tubes = [tube1, tube2];
%tubes = [tube1, tube2, tube3];

robot = Robot(tubes);

[rho_m, theta_rad] = robot.invkinematics(q_initial, goal_pos);
rho = rho_m*1000
theta = rad2deg(theta_rad)

% Verification
[phi_ref, k_ref] = robot.calculate_phi_and_kappa(theta_ref)
[ll_ref] = robot.get_links(rho_ref)
T_result_ref_check = robot.fkin(q_ref)
T_result = robot.fkin([rho, theta])

%% MOVE
% Move robot for 2 tube case
%drive_bot.travel_for(rho(1), rho(2), 0, theta(1), theta(2), 0)

