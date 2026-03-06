%% RBE 522 - Continuum Robotics, A Term 2022
% Inverse Kinematics of a Continuum Robot
clear
close all
clc
%% Create a robot with 3 links

OD = [1.8 1.6 1.4]; % link diameters [mm]

tube1 = Tube(3.046*10^-3, 3.3*10^-3, 1/17, 90*10^-3, 50*10^-3, 1935*10^6);
tube2 = Tube(2.386*10^-3, 2.64*10^-3, 1/22, 170*10^-3, 50*10^-3, 1935*10^6);
tube3 = Tube(1.726*10^-3, 1.98*10^-3, 1/29, 250*10^-3, 50*10^-3, 1935*10^6);

% TODO You will need to uncomment one of the below lines, depending if you
% are testing a 2-tube or 3-tube case
%tubes = [tube1, tube2];
tubes = [tube1, tube2, tube3];

robot = Robot(tubes);
%% 


q_var = [%0, 0, 0, 0, 0, 0;
         %20, 50, 70, 45, -45, 45]; 
         0, 0, 0, 0, 0, 0];
% q_var = [20,50,45,10]
T = {};
for i = 1:size(q_var,1)
    disp(q_var(i,:));
    set_T = robot.fkin(q_var(i,:));
    T{i} = set_T;
    disp(T{i});
end
first_p=set_T(1:3,4)
guess=1.*ones(size(q_var))
newQ=robot.calculate_ik(set_T,guess)

newT=robot.fkin(newQ)
second_p=newT(1:3,4)
%% 
guess=1.*ones(1,6)
set_T=eye(4)
set_T(1:3,4)=[80;80;0]
Q=robot.calculate_ik(set_T,guess)
robot.fkin(Q)

%% Create a configuration c and plot the robot pose
% kappa1 = robot.kappa(1)*10^-3 % curvature of link 1 [m^-1]
% phi1   = robot.phi(1)     % base rotation of link 1 [rad]
% l1     = robot.lls(1)*10^3;     % length of link 1 [m]
% 
% kappa2 = robot.kappa(2)*10^-3 % curvature of link 2 [m^-1]
% phi2   = robot.phi(2)     % base rotation of link 2 [rad]
% l2     = robot.lls(2)*10^3;     % length of link 2 [m]
% 
% if robot.num_tubes==3
%     kappa3 = robot.kappa(3)*10^-3 % curvature of link 3 [m^-1]
%     phi3   = robot.phi(3)     % base rotation of link 3 [rad]
%     l3     = robot.lls(3)*10^3;     % length of link 3 [m]
% end
nLinks=robot.num_tubes

c=zeros(1,3*robot.num_tubes)
for i=1:robot.num_tubes
    k=robot.kappa(i)*10^-3;
    p=robot.phi(i)
    l=robot.lls(i)*10^3
    c((i-1)*3+1:(i-1)*3+3)=[k p l]
end

% Robot Configuration


% c = [kappa1 phi1 l1 kappa2 phi2 l2 kappa3 phi3 l3]';


% Plot
[T, links] = fwkinematics(c);

figure, axis equal, grid on, hold on
colorPalette =    [1.0000    0.1034    0.7241;
                   1.0000    0.8276         0;
                   0    0.3448         0];

worldRef = triad('Scale', 3, 'linewidth', 5);
robotRef = triad('matrix', T, 'Scale', 3, 'linewidth', 5);
xlabel('X [mm]', 'fontsize', 24)
ylabel('Y [mm]', 'fontsize', 24)
zlabel('Z [mm]', 'fontsize', 24);
% xlim([-7 7]), ylim([-7 7]), zlim([0 20]);
title('Position control of a 3-link continuum robot', 'fontsize', 16);
set(gca, 'FontSize', 24);
view(110.4, 33.2);

s = cell(nLinks);

for jj = 1 : nLinks
    [X, Y, Z] = gencyl(links{jj}, OD(jj) / 2 * ones(1,size(links{jj}, 2)));
    s{jj} = surf(X, Y, Z);
    set(s{jj}, 'FaceColor', colorPalette(jj,:), 'FaceAlpha', 1);
end

%% Define a path in task space

t = 10:pi/5:10*pi;
st = linspace(1,5,size(t,2)).*sin(t);
ct = linspace(1,5,size(t,2)).*cos(t);

trajectory = [st; ct; t/2];

% Plot the path
scatter3(trajectory(1,:), trajectory(2,:), trajectory(3,:),'filled');

%% Inverse Kinematics
for i = 1:size(trajectory, 2)
    p_target = trajectory(:, i);
    
    [T_curr, ~] = fwkinematics(c);
    p_curr = T_curr(1:3, 4);
        
    err = p_target - p_curr;
        
    J = jacob0(c);
        
    J_omega = J(1:3, :);
    J_v_space = J(4:6, :);
        
    p_skew = [ 0,         -p_curr(3),  p_curr(2);
               p_curr(3),  0,         -p_curr(1);
              -p_curr(2),  p_curr(1),  0        ];
              
    J_v = J_v_space - p_skew * J_omega;
       
    J_pinv = pinv(J_v); 
    delta_c = J_pinv * err;
        
    c = c + delta_c;
    
    % Animation
    [T, links] = fwkinematics(c);
    
    for jj = 1:nLinks
        [X, Y, Z] = gencyl(links{jj}, OD(jj) / 2 * ones(1,size(links{jj}, 2)));
        set(s{jj}, 'XData', X, 'YData', Y, 'ZData', Z);
    end
    
    set(robotRef, 'Matrix', T);
    drawnow;
end


%% Ancillary functions
function [T,links] = fwkinematics(c)
    nLinks = length(c)/3;

    T = eye(4);
    links = cell(1,nLinks);

    for ii = 0 : nLinks - 1
        [Tii, link] = arckinematics([c(ii*3+1), c(ii*3+2), c(ii*3+3)]);
        links{ii+1} = applytransform(link, T);
        T = T * Tii;
    end
end
