%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% controller: PD controller for UAV position and yaw
%
% Syntax: commands = controller(pose, pose_d, velocity_d)
%
% Inputs:
%    - pose: actual pose ([x y z roll pitch yaw]) of UAV
%    - pose_d: desired pose ([x* y* z* yaw*]) of UAV
%    - velocity_d: desired velocity ([v_x* v_y* v_z* w_yaw*]) of UAV
%
% Outputs:
%    - commands: control commands ([roll* pitch* yaw* thrust*]) to UAV
%
% Example: 
%    command = controller(pose, pose_d, velocity_d);
%
% m-files required: none
% mat-files required: none
% other files required: none
%
% Author: Andriy Sarabakha
% email: andriyukr@gmail.com
% Website: http://www.sarabkha.info
% Last revision: 08/02/2021
% Environment: MATLAB R2020b
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

function [commands, error] = controller(pose, pose_d, velocity_d)

global dt initial_state;

persistent old_pose; % stores previous pose
persistent e_x_sum e_y_sum e_z_sum e_x_pre e_y_pre e_z_pre
      
if isempty(old_pose)
    old_pose = initial_state(1:6);
    e_x_sum = 0; e_x_pre = 0;
    e_y_sum = 0; e_y_pre = 0;
    e_z_sum = 0; e_z_pre = 0;
end

%% Initialize gains

Kp_x = 1;%100;
Ki_x = 100;
Kd_x = 2;%50;

Kp_y = 1;%100;
Ki_y = 100;
Kd_y = 2;%50;

Kp_z = 10; %50;
Ki_z = 100;
Kd_z = 1; %10;

%% Actual state

x = pose(1);
y = pose(2);
z = pose(3);
yaw = pose(6);

vx = (pose(1) - old_pose(1))/dt;
vy = (pose(2) - old_pose(2))/dt;
vz = (pose(3) - old_pose(3))/dt;

old_pose = pose;

%% Reference values

x_ref = pose_d(1);
y_ref = pose_d(2);
z_ref = pose_d(3);
yaw_ref = pose_d(4);

dx_ref = velocity_d(1);
dy_ref = velocity_d(2);
dz_ref = velocity_d(3);

%% Compute errors

e_x = cos(yaw)*(x_ref - x) - sin(yaw)*(y_ref - y);
e_dx = cos(yaw)*(dx_ref - vx) - sin(yaw)*(dy_ref - vy);

e_y = sin(yaw)*(x_ref - x) + cos(yaw)*(y_ref - y);
e_dy = sin(yaw)*(dx_ref - vx) + cos(yaw)*(dy_ref - vy);

e_z = z_ref - z;
e_dz = dz_ref - vz;

[lookup_table_p1, lookup_table_i1] = fuzzy_att_control_surface();
[lookup_table_p2, lookup_table_i2] = fuzzy_alt_control_surface();

% [lookup_table_p1, lookup_table_d1] = fuzzy_att_control_surface();
% [lookup_table_p2, lookup_table_d2] = fuzzy_alt_control_surface();

[IS1_x, IS2_x] = find_att_defuzz_angle(e_x, e_x_sum*dt);
[IS1_y, IS2_y] = find_att_defuzz_angle(e_y, e_y_sum*dt);
[IS1_z, IS2_z] = find_alt_defuzz_angle(e_z, e_z_sum*dt);
% disp(e_x_sum)
disp(IS1_x)

% [IS1_x, IS2_x] = find_att_defuzz_angle(e_x, e_dx);
% [IS1_y, IS2_y] = find_att_defuzz_angle(e_y, e_dy);
% [IS1_z, IS2_z] = find_alt_defuzz_angle(e_z, e_dz);

Kp_f_x = lookup_table_p1(IS1_x, IS2_x);
Kp_f_y = lookup_table_p1(IS1_y, IS2_y);
Kp_f_z = lookup_table_p2(IS1_z, IS2_z);

Ki_f_x = lookup_table_i1(IS1_x, IS2_x);
Ki_f_y = lookup_table_i1(IS1_y, IS2_y);
Ki_f_z = lookup_table_i2(IS1_z, IS2_z);

% Kd_f_x = lookup_table_d1(IS1_x, IS2_x);
% Kd_f_y = lookup_table_d1(IS1_y, IS2_y);
% Kd_f_z = lookup_table_d2(IS1_z, IS2_z);

Kp_x = Kp_x+Kp_f_x; Kp_y = Kp_y+Kp_f_y; Kp_z = Kp_z+Kp_f_z;
Ki_x = Ki_x+Ki_f_x; Ki_y = Ki_y+Ki_f_y; Ki_z = Ki_z+Ki_f_z;

% Kp_x = Kp_x+Kp_f_x; Kp_y = Kp_y+Kp_f_y; Kp_z = Kp_z+Kp_f_z;
% Kd_x = Kd_x+Kd_f_x; Kd_y = Kd_y+Kd_f_y; Kd_z = Kd_z+Kd_f_z;

%% Position controller

e_x_sum = e_x + e_x_sum;
e_y_sum = e_y + e_y_sum;
e_z_sum = e_z + e_z_sum;

if e_x_sum>5
    e_x_sum = 5;
elseif e_x_sum<-5
    e_x_sum = -5;
end

if e_y_sum>5
    e_y_sum = 5;
elseif e_y_sum<-5
    e_y_sum = -5;
end

if e_z_sum>5
    e_z_sum = 5;
elseif e_z_sum<-5
    e_z_sum = -5;
end
roll_ref = Kp_y*e_y + Ki_y*e_y_sum*dt; %
pitch_ref = -Kp_x*e_x - Ki_x*e_x_sum*dt; %
thrust = 1.1*9.81 + ...     % hower thrust = mass*gravity
         Kp_z*e_z + Ki_z*e_z_sum*dt;   % total thrust on the body along z-axis

% roll_ref = Kp_y*e_y + Kd_y*e_dy; %
% pitch_ref = -Kp_x*e_x - Kd_x*e_dx; %
% thrust = 1.1*9.81 + ...     % hower thrust = mass*gravity
%            Kp_z*e_z + Kd_z*e_dz;   % total thrust on the body along z-axis

roll_ref = max([min([roll_ref pi/2]) -pi/2]); %
pitch_ref = max([min([pitch_ref pi/2]) -pi/2]); %

commands = [roll_ref pitch_ref yaw_ref thrust];
error = [e_x e_y e_z e_dx e_dy e_dz];

% e_x_pre = e_x;
% e_y_pre = e_y;
% e_z_pre = e_z;
end