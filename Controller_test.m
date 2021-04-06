clc
clear all
close all

global dt initial_state;
simulation_duration = 60; % [s]

dt = 0.001;
kend = simulation_duration/dt;

initial_state = [1 1 1 0 0 pi 0 0 0 0 0 0];
               %[x y z roll pitch yaw vx vy vz p q r]

pose = zeros(kend, 6);


pose(1,:) = initial_state(1:6);

t = dt*(1:kend)';
                              
control_uav = controller();

for k = 1:kend
    command = control_uav.control_run(pose(k,:), [0 0 0 0], [0.1 0.1 0.1 0.1], [0.1 0.1 0.1 0.1]);
    pose(k,:) = uav(command);
end

figure(1)
plot(t,pose(:,1),'r')
hold on
plot(t,pose(:,2),'b')
plot(t,pose(:,3),'g')
hold off