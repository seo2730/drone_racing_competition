
function input = controller(pose, pose_d, velocity_d, accel_d)
    %CONTROLLER 이 함수의 요약 설명 위치
    %   자세한 설명 위치

    persistent old_pose;
    persistent old_v_de; % old velocity desired
    if isempty(old_pose)
         old_pose = pose;
    end
   %%%%%%% Parameter %%%%%%%%   
   % Quadrotor specs
    Ixx = 8.1*10^(-3);  % Quadrotor moment of inertia around X axis  (X축의 관성 모멘트)
    Iyy = 8.1*10^(-3);  % Quadrotor moment of inertia around Y axis  (Y축의 관성 모멘트)
    Izz = 14.2*10^(-3); % Quadrotor moment of inertia around Z axis (Z축의 관성 모멘트)
    m = 1.1;            % Mass of the Quadrotor in Kg(드론 무게, 단위 : kg)
    g = 9.81;           % Gravitational acceleration (중력 가속도)
    l = 0.23;
    b = 3.1*10^(-5);
    D = 7.5*10^(-7);
    lambda1 = 0.01;
    lambda2 = 0.01;
    gamma = 0.1; % 상현 수정
    tilde = 5; % 상현 수정
    simga = 0.3; % 상현 수정
    dt = 0.01;

    % Sliding control
    %cx = 1; J0 = 0.1; J1 = 0.1; pix1 = 1; pix2 = 1; % design parameter in x
    %cy = 1; J2 = 0.1; J3 = 0.1; piy1 = 1; piy2 = 1; % design parameter in y
    %cz = 1; J4 = 0.1; J5 = 0.1; piz1 = 1; piz2 = 1; % design parameter in z
    
    %%%%%%%%%%%%%%%%%%%%%%%%%
    
    % 위치 제어 : input : 현재 위치, desired 위치, 속도, 가속도
    [d_att, d_thrust, old_v_de, old_pose] = position_controller(pose, pose_d, velocity_d, accel_d, old_pose, old_v_de);

    % attitude controller에서 현재 각속도가 필요함 
    pitch_dot = (pose(:,4) - old_pose(:,4))/dt;
    roll_dot = (pose(:,5) - old_pose(:,5))/dt;
    yaw_dot = (pose(:,6) - old_pose(:,6))/dt;
    old_pose = pose;
    att_dot = [pitch_dot roll_dot yaw_dot];
    % 자세 제어 : 현재 자세, 각속도. desired 자세, 각속도, mode(에러가 발생함)
    % Constraint 필요
    input_roll = max([min([attitude_control_run(pose(:,4:6),att_dot,d_att(1:3),d_att(4:6),1) 3.1416/2]) -3.1416/2]);
    input_pitch = max([min([attitude_control_run(pose(:,4:6),att_dot,d_att(1:3),d_att(4:6),2) 3.1416/2]) -3.1416/2]);
    input_yaw = attitude_control_run(pose(:,4:6),att_dot,d_att(1:3),d_att(4:6),3);

    d_thrust = max([min([d_thrust,  m*g*10]), m*g]);
    input = [input_roll, input_pitch, input_yaw, d_thrust] ; 
end

function [d_att, d_thrust, old_v_de, old_pose] = position_controller(pose, pose_d, velocity_d, accel_d, old_pose, old_v_de)
    %global dt initial_state; % 생성자로 global 안쓰기 %%%%%%%%%%% 수정하기
    %persistent old_pose; % stores previous pose % 생성자로 persistent 안쓰기 %%%%%%%%%%% 수정하기
    persistent dt J c m
    persistent e1x_int e1y_int e1z_int
    persistent Kax Kay Kaz pix piy piz 
    if isempty(dt)
        m = 1.1; 
        dt = 0.01;
        J = [0.1, 0.1, 0.1, 0.1, 0.1, 0.1];
        c = [1, 1, 1];
        e1x_int = 0; e1y_int = 0; e1z_int = 0;
        Kax = 0.012; Kay = 0.012; Kaz = 0.012;
        pix = [1,1]; piy = [1,1]; piz = [1, 1];
    end
    

    x = pose(1);
    y = pose(2);
    z = pose(3);
    yaw = pose(6);

    vx = (pose(1) - old_pose(1))/dt;
    vy = (pose(2) - old_pose(2))/dt;
    vz = (pose(3) - old_pose(3))/dt;

    ax = (pose(4) - old_pose(7))/dt;
    ay = (pose(5) - old_pose(8))/dt;
    az = (pose(6) - old_pose(9))/dt;

    x_ref = pose_d(1);
    y_ref = pose_d(2);
    z_ref = pose_d(3);
    yaw_ref = pose_d(4);

    dx_ref = velocity_d(1);
    dy_ref = velocity_d(2);
    dz_ref = velocity_d(3);

    dxdx_ref = accel_d(1);
    dydy_ref = accel_d(2);
    dzdz_ref = accel_d(3);

    if isempty(old_v_de)
        old_v_de =[dx_ref, dy_ref, dz_ref];
    end  
    %%%% Position SMC Control %%%%

    % X
    e1x = x - x_ref;
    dot_e1x = vx - dx_ref;
    e1x_int = e1x_int + e1x;
    beta1 = dx_ref-old_v_de(1)-J(1)*e1x_int*dt-J(2)*e1x;
    e2x = vx - beta1; % B를 넣어야할지? Desired velocity로 넣어야할지? 

    sx = c(1)*e1x + e2x;

    Ux = Kax*dxdx_ref+m*(dxdx_ref + c(1)*(J(2)*e1x + J(1)*e1x*dt - e2x) ...
         - (J(2)*dot_e1x + J(1)*e1x) - pix(1)*sign(sx) + pix(2)*sx);

    % Y
    e1y = y - y_ref;
    dot_e1y = vy - dy_ref;
    e1y_int = e1y_int + e1y;
    beta2 = dy_ref-old_v_de(2)-J(3)*e1y_int*dt-J(4)*e1y;
    e2y = vy - beta2; % B를 넣어야할지? Desired velocity로 넣어야할지? 

    sy = c(2)*e1y + e2y;

    Uy = Kay*dydy_ref+m*(dydy_ref + c(2)*(J(4)*e1y + J(3)*e1y*dt - e2y) ...
         - (J(4)*dot_e1y + J(3)*e1y) - piy(1)*sign(sy) + piy(2)*sy);

    % Z
    e1z = z - z_ref;
    dot_e1z = vz - dz_ref;
    e1z_int = e1z_int + e1z;
    beta3 = dz_ref-old_v_de(3)-J(5)*e1z_int*dt-J(6)*e1z;
    e2z = vz - beta3; % B를 넣어야할지? Desired velocity로 넣어야할지? 

    sz = c(3)*e1z + e2z;

    Uz = Kaz*dzdz_ref+m*(dzdz_ref + c(3)*(J(6)*e1z + J(5)*e1z*dt - e2z) ...
         - (J(6)*dot_e1z + J(5)*e1z) - piz(1)*sign(sz) + piz(2)*sz);

    roll = atan((Ux*cos(yaw) + Uy*sin(yaw))/Uz);
    pitch = atan(cos(roll)*(Ux*sin(yaw) - Uy*cos(yaw))/Uz);
    d_thrust = Uz/(cos(roll)*cos(pitch));

    pitch_dot = (pitch - old_pose(4))/dt;
    roll_dot = (roll - old_pose(5))/dt;
    yaw_dot = (yaw - old_pose(6))/dt;

    old_pose = pose;
    old_v_de = [dx_ref, dy_ref, dz_ref];

    d_att = [pitch roll yaw pitch_dot roll_dot yaw_dot];

end

function input = attitude_control_run(att, att_dot, att_d, att_d_dot, mode)
    global sigma
    persistent y_dot_pre pi
    persistent lambda1 lambda2 gamma tilde dt
    if isempty(y_dot_pre)
        y_dot_pre = 0;
        lambda1 = 0.01; lambda2 = 0.01; gamma=0.1; tilde = 5; dt= 0.01;
        pi = 0;
    end
    x = [att(1), att_dot(1), att(2), att_dot(2), att(3), att_dot(3)]';
    y = att(mode);
    y_d = att_d(mode);
    y_dot_d = att_d_dot(mode);
    y_dot = att_dot(mode);
    y_dotdot = (y_dot - y_dot_pre)/dt;
    e1 = y_d - y;
    e1_dot = y_dot_d - y_dot;
    alpha = y_dot_d + lambda1 * e1;
    e2 = y_dot - alpha ;
    Z = [x',y,y_dot,y_dotdot];
    fuzzy_basis = fuzzy_basis_function(Z);
    neural_basis = neural_basis_function(Z);
    basis = [fuzzy_basis, neural_basis];
    %%%%%% pi 
    % basis'*basis에서 basis*basis'로 수정 (상현)
    pi_dot  = (gamma/(2*tilde^2)) * e2^2 * (basis* basis') - gamma*sigma*pi;
    pi = pi + pi_dot*dt;
    input = -lambda2 * e2 - (1/(2*tilde^2))*e2*pi * (basis * basis') ;
    y_dot_pre = y_dot;
end

function fuzzy_output = fuzzy_membership(x,num)
    switch num
        case 1
            fuzzy_output =  1/(1+exp(3*pi/2*(x+2*pi/3)));
        case 2
            fuzzy_output = exp(-((x+pi/6)/(pi/3))^2);
        case 3
            fuzzy_output = exp(-((x/(pi/3))^2));
        case 4
            fuzzy_output = exp(-((x-pi/6)/(pi/3))^2);
        case 5
            fuzzy_output =  1/(1+exp(-3*pi/2*(x-2*pi/3)));
    end
end

function fuzzy_basis = fuzzy_basis_function(x)
    membership_num = 5;
    pi_mu = 1;
    sum = 0;
    fuzzy_variable = x;
    pi_mu_array = zeros(1,membership_num);
    for j = 1:membership_num
        for eta = 1:size(fuzzy_variable,1)
            mu=fuzzy_membership(fuzzy_variable(eta),j);
            pi_mu = pi_mu*mu;
        end
        pi_mu_array(j) = pi_mu;
        sum = sum+pi_mu;
    end
    fuzzy_basis = pi_mu_array/sum;
end

function neural_basis = neural_basis_function(x)
    v = [-2 -1 0 1 2];
    global sigma
    sigma = 0.03;
    neural_basis_array = zeros(1,size(v,2));
    for j = 1:size(v,2)
        neural_basis_array(j) = exp(-1*norm(x-v(j))^2 / sigma);
    end
    neural_basis = neural_basis_array;
end

