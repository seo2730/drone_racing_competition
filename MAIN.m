%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% MAIN: main file for the autonomous drone racing simulation
%
% m-files required:
%    - controller
%    - environment 
%    - trajectory 
%    - uav
% mat-files required: none
% other files required:
%    - gates.txt (in /gates): contains poses of the gates
%
% Author: Andriy Sarabakha
% email: andriyukr@gmail.com
% Website: http://www.sarabkha.info
% Last revision: 08/02/2021
% Environment: MATLAB R2020b
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%% Clean the workspace

clc
clear all
close all

global dt initial_state;

%% Changeable parameters

simulation_duration = 60; % [s]

%% Static parameters

dt = 0.001;
kend = simulation_duration/dt;

initial_state = [0 0 0.1 0 0 pi 0 0 0 0 0 0];%[0 0 0.1 0 0 pi 0 0 0 0 0 0];

%% Prealocate variables

pose = zeros(kend, 6);
pose(1,:) = initial_state(1:6);

t = dt*(1:kend)';

%% Read gates' poses

gates = load('gates/gates.txt');
gates(:,4) = gates(:,4)/180*pi; % converts from degrees to radiants

%% Trajectory genaration

[pose_d, velocity_d] = trajectory(gates);
saved_command = zeros(4, kend);
%% Main loop

elapsed = 0;
for k = 1:kend
    
    %% UAV controller
    tic;
    
%     [command,error] = controller(pose(k,:), pose_d(k,:), velocity_d(k,:));
    [command,error] = controller(pose(k,:), pose_d(k,:), velocity_d(k,:));
    saved_command(:,k) = command;
    saved_error(:,k) = error;
    elapsed = elapsed + toc; % for computational time
    
    %% UAV model
    
    [pose(k + 1,:),error_att(k,:)] = uav(command);
    error_pos(k,:) = pose_d(k,1:3) - pose(k,1:3);
%     error_att(k,:) = saved_command(1:3,k)'- pose(k,4:6)  ;
end

%% Show animation

score = environment(gates, pose, pose_d);

%% Plot
figure(2)
plot(t,error_pos(:,1),'r')
hold on
plot(t,error_pos(:,2),'b')
plot(t,error_pos(:,3),'k')
hold off
legend('x error','y error','z error')
axis([0 60 -1.5 1.5])

figure(3)
tiledlayout(3,1)
nexttile
plot(t,error_att(:,1),'r')
legend('Roll error')
axis([0 60 -2 2])
nexttile
plot(t,error_att(:,2),'b')
legend('Pitch error')
axis([0 60 -2 2])
nexttile
plot(t,error_att(:,3),'k')
legend('Yaw error')
axis([0 60 -0.5 0.5])

figure(4)
plot(t,saved_error(1,:),'r')
hold on
plot(t,saved_error(2,:),'b')
plot(t,saved_error(3,:),'k')
hold off
legend('yaw * x error',' yaw * y error','yaw * z error')
axis([0 60 -1.5 1.5])


%% Show results

% disp('**********');
% disp(['Controller runs at ', num2str(kend/elapsed), 'Hz']);
% disp(['Score is ', num2str(score)]);