function input = controller(pose, pose_d, velocity_d, accel_d)
    %CONTROLLER 이 함수의 요약 설명 위치
    %   자세한 설명 위치

    persistent old_pose roll_dot pitch_dot yaw_dot p q r input_roll input_pitch input_yaw;
    persistent old_v_de; % old velocity desired
    if isempty(old_pose)
         old_pose = [0 0 0 0 0 0];
         roll_dot=0; pitch_dot=0; yaw_dot=0;
         p=0; q=0; r=0;
         input_roll=0; input_pitch=0; input_yaw=0;
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
    dt = 0.001;
    
    %%%%%%%%%%%%%%%%%%%%%%%%% 
    
    % 위치 제어 : input : 현재 위치, desired 위치, 속도, 가속도
    [d_att, d_thrust, old_v_de, old_pose] = position_controller(pose, pose_d, velocity_d, accel_d, old_pose, old_v_de,input_roll,input_pitch,input_yaw);
    
    % attitude controller에서 현재 각속도가 필요함 
%     roll_dot = (pose(:,4) - old_pose(:,4))/dt;
%     pitch_dot = (pose(:,5) - old_pose(:,5))/dt;
%     yaw_dot = (pose(:,6) - old_pose(:,6))/dt;
    p_dot = ((Iyy - Izz)/Ixx)*q*r + (input_roll/Ixx);
    q_dot = ((Izz - Ixx)/Iyy)*p*r + (input_pitch/Iyy);
    r_dot = ((Ixx - Iyy)/Izz)*p*q + (input_yaw/Izz);
    
    p = p + dt*p_dot;
    q = q + dt*q_dot;
    r = r + dt*r_dot;
    
    roll_dot = p + sin(pose(:,4))*tan(pose(:,5))*q + cos(pose(:,4))*tan(pose(:,5))*r;
    pitch_dot = cos(pose(:,4))*q - sin(pose(:,4))*r;  
    yaw_dot = sin(pose(:,4))/cos(pose(:,5))*q + cos(pose(:,4))/cos(pose(:,5))*r;
%     roll_dot = roll_dot + dt*p_dot;
%     pitch_dot = pitch_dot + dt*q_dot;  
%     yaw_dot = yaw_dot + dt*r_dot;
%     old_pose = pose;
    att_dot = [roll_dot pitch_dot yaw_dot];
    
    % 자세 제어 : 현재 자세, 각속도. desired 자세, 각속도, mode
    % Constraint 필요
    input_roll = max([min([attitude_control_run(pose(:,4:6),att_dot,d_att(1:3),d_att(4:6),1) 3.1416/2]) -3.1416/2]);
    input_pitch = max([min([attitude_control_run(pose(:,4:6),att_dot,d_att(1:3),d_att(4:6),2) 3.1416/2]) -3.1416/2]);
    input_yaw = attitude_control_run(pose(:,4:6),att_dot,d_att(1:3),d_att(4:6),3);

%     d_thrust = max([min([d_thrust  g*10]) 0]);
    input = [input_roll, input_pitch, input_yaw, d_thrust] ; 
end

function [d_att, d_thrust, old_v_de, old_pose] = position_controller(pose, pose_d, velocity_d, accel_d, old_pose, old_v_de, input_roll, input_pitch, input_yaw)
    %global dt initial_state; % 생성자로 global 안쓰기 %%%%%%%%%%% 수정하기
    %persistent old_pose; % stores previous pose % 생성자로 persistent 안쓰기 %%%%%%%%%%% 수정하기
    persistent dt J c m g thrust_past vx vy vz 
    persistent roll_dot pitch_dot yaw_dot p q r
    persistent e1x_int e1y_int e1z_int 
    persistent Kax Kay Kaz pix piy piz 
    if isempty(dt)
        m = 1.1; 
        dt = 0.001;
        J = [0.1, 0.1, 0.1, 0.1, 1, 1];
        c = [0.1, 0.1 1];
        e1x_int = 0; e1y_int = 0; e1z_int = 0;
        Kax = 0.1; Kay = 0.1; Kaz = 1;
        pix = [0.1,0.1]; piy = [0.1,0.1]; piz = [1,1];
        g=9.81;
        thrust_past=m*g;
        vx = 0; vy = 0; vz = 0;
        roll_dot=0; pitch_dot=0; yaw_dot=0;
        p=0; q=0; r=0;
    end
    
    Ixx = 8.1*10^(-3);  % Quadrotor moment of inertia around X axis  (X축의 관성 모멘트)
    Iyy = 8.1*10^(-3);  % Quadrotor moment of inertia around Y axis  (Y축의 관성 모멘트)
    Izz = 14.2*10^(-3); % Quadrotor moment of inertia around Z axis (Z축의 관성 모멘트)

    x = pose(1);
    y = pose(2);
    z = pose(3);
    yaw = pose(6);
%     yaw = pose_d(4);
%     x = pose_d(1);
%     y = pose_d(2);
%     z = pose_d(3);
%     yaw = pose_d(4);

%     vx = (pose(1) - old_pose(1))/dt;
%     vy = (pose(2) - old_pose(2))/dt;
%     vz = (pose(3) - old_pose(3))/dt;

    ax = (cos(pose(4))*cos(pose(6))*sin(pose(5)) + sin(pose(4))*sin(pose(6)))*(thrust_past/m);
    ay = (-cos(pose(4))*sin(pose(5))*sin(pose(6)) + cos(pose(6))*sin(pose(4)))*(thrust_past/m);
    az = (cos(pose(5))*cos(pose(4)))*(thrust_past/m) - g;

    vx = vx + ax*dt;
    vy = vy + ay*dt;
    vz = vz + az*dt;

    x_ref = pose_d(1);
    y_ref = pose_d(2);
    z_ref = pose_d(3);
    yaw_ref = pose_d(4);
    if(abs(yaw_ref - yaw) > pi)
        if(yaw_ref < yaw)
            yaw = yaw - 2*pi;
        else
            yaw = yaw + 2*pi;
        end
    end

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
%     beta1 = dx_ref-old_v_de(1)-J(1)*e1x_int*dt-J(2)*e1x;
    beta1 = dx_ref-J(1)*e1x_int*dt-J(2)*e1x;
    e2x = vx - beta1; % B를 넣어야할지? Desired velocity로 넣어야할지? 

    sx = c(1)*e1x + e2x;

    Ux = Kax*dxdx_ref+m*(dxdx_ref + c(1)*(J(2)*e1x + J(1)*e1x*dt - e2x) ...
         - (J(2)*dot_e1x + J(1)*e1x) - pix(1)*sign(sx) + pix(2)*sx);

    % Y
    e1y = y - y_ref;
    dot_e1y = vy - dy_ref;
    e1y_int = e1y_int + e1y;
%     beta2 = dy_ref-old_v_de(2)-J(3)*e1y_int*dt-J(4)*e1y;
    beta2 = dy_ref-J(3)*e1y_int*dt-J(4)*e1y;
    e2y = vy - beta2; % B를 넣어야할지? Desired velocity로 넣어야할지? 

    sy = c(2)*e1y + e2y;

    Uy = Kay*dydy_ref+m*(dydy_ref + c(2)*(J(4)*e1y + J(3)*e1y*dt - e2y) ...
         - (J(4)*dot_e1y + J(3)*e1y) - piy(1)*sign(sy) + piy(2)*sy);

    % Z
    e1z = z - z_ref;
    dot_e1z = vz - dz_ref;
    e1z_int = e1z_int + e1z;
%     beta3 = dz_ref-old_v_de(3)-J(5)*e1z_int*dt-J(6)*e1z;
    beta3 = dz_ref-J(5)*e1z_int*dt-J(6)*e1z;
    e2z = vz - beta3; % B를 넣어야할지? Desired velocity로 넣어야할지? 

    sz = c(3)*e1z + e2z;

    Uz = Kaz*dzdz_ref+m*(dzdz_ref + c(3)*(J(6)*e1z + J(5)*e1z*dt - e2z) ...
         - (J(6)*dot_e1z + J(5)*e1z) - piz(1)*sign(sz) + piz(2)*sz) + m*g;

    pitch = atan((Ux*cos(yaw) + Uy*sin(yaw))/Uz);
    roll = atan(cos(pitch)*(Ux*sin(yaw) - Uy*cos(yaw))/Uz);
    d_thrust = 0.55*9.81 + Uz/(cos(roll)*cos(pitch));
%     pitch = atan((Ux*cos(yaw_ref) + Uy*sin(yaw_ref))/Uz);
%     roll = atan(cos(pitch)*(Ux*sin(yaw_ref) - Uy*cos(yaw_ref))/Uz);
%     d_thrust = Uz/(cos(roll)*cos(pitch));
    thrust_past = d_thrust;

%     roll_dot = (roll - old_pose(4))/dt;
%     pitch_dot = (pitch - old_pose(5))/dt;
%     yaw_dot = (yaw - old_pose(6))/dt;
    p_dot = ((Iyy - Izz)/Ixx)*q*r + (input_roll/Ixx);
    q_dot = ((Izz - Ixx)/Iyy)*p*r + (input_pitch/Iyy);
    r_dot = ((Ixx - Iyy)/Izz)*p*q + (input_yaw/Izz);
    
    p = p + dt*p_dot;
    q = q + dt*q_dot;
    r = r + dt*r_dot;
    
    roll_dot = p + sin(pose(:,4))*tan(pose(:,5))*q + cos(pose(:,4))*tan(pose(:,5))*r;
    pitch_dot = cos(pose(:,4))*q_dot - sin(pose(:,4))*r;  
    yaw_dot = sin(pose(:,4))/cos(pose(:,5))*q + cos(pose(:,4))/cos(pose(:,5))*r;
%     roll_dot = roll_dot + dt*p_dot;
%     pitch_dot = pitch_dot + dt*q_dot;  
%     yaw_dot = yaw_dot + dt*r_dot;
    
    old_pose = pose;
    old_v_de = [dx_ref, dy_ref, dz_ref];

    d_att = [roll pitch  yaw roll_dot pitch_dot yaw_dot];

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

