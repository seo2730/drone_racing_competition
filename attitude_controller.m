classdef attitude_controller < handle
    %UNTITLED2 이 클래스의 요약 설명 위치
    %   자세한 설명 위치
    
    properties
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
        gamma = 0.01;
        tilde = 0.01;
        simga = 0.01;
        dt = 0.01;
        
        % Sliding control
%         cx = [-1 -0. 0 0.5 1]; J0 = 1; J1 = 1; pix1 = 1; pix2 = 1; % design parameter in x
%         cy = [-1 -0. 0 0.5 1]; J2 = 1; J3 = 1; piy1 = 1; piy2 = 1; % design parameter in y
%         cz = [-1 -0. 0 0.5 1]; J4 = 1; J5 = 1; piz1 = 1; piz2 = 1; % design parameter in z
        cx = 0.001; J0 = 0.001; J1 = 0.001; pix1 = 0.001; pix2 = 0.001; % design parameter in x
        cy = 0.001; J2 = 0.001; J3 = 0.001; piy1 = 0.001; piy2 = 0.001; % design parameter in y
        cz = 0.001; J4 = 0.001; J5 = 0.001; piz1 = 0.001; piz2 = 0.001; % design parameter in z

    end
    
    methods
        function obj = attitude_controller() %생성자
           
        end   
        
        function input = controller_run(obj, pose, pose_d, velocity_d, accel_d)
           % 전역변수 : dt, 초기값
           global dt initial_state
           % 위치 제어 : input : 현재 위치, desired 위치, 속도, 가속도
           d_att = obj.position_controller(pose, pose_d, velocity_d, accel_d);
           disp(d_att)
           persistent old_pose;
           if isempty(old_pose)
                old_pose = initial_state(1:9);
           end
           % attitude controller에서 현재 각속도가 필요함 
           phi_dot = (pose(:,4) - old_pose(:,4))/dt;
           theta_dot = (pose(:,5) - old_pose(:,5))/dt;
           psi_dot = (pose(:,6) - old_pose(:,6))/dt;
           old_pose = pose;
           att_dot = [phi_dot theta_dot psi_dot];
           % 자세 제어 : 현재 자세, 각속도. desired 자세, 각속도, mode
           input = obj.attitude_control_run(pose(:,4:6),att_dot,d_att(1:3),d_att(4:6),1);
%            input(2) = obj.attitude_control_run(pose(:,4:6),att_dot,d_att(1:3),d_att(4:6),2);
%            input(3) = obj.attitude_control_run(pose(:,4:6),att_dot,d_att(1:3),d_att(4:6),3);
%            input(4) = d_att(7);
        end
        
        function d_att = position_controller(obj, pose, pose_d, velocity_d, accel_d)
            global dt initial_state;
            persistent old_pose; % stores previous pose
            
            if isempty(old_pose)
                old_pose = initial_state(1:9);
            end
                 
            x = pose(1);
            y = pose(2);
            z = pose(3);
            yaw = pose(6);

            vx = (pose(1) - old_pose(1))/dt;
            vy = (pose(2) - old_pose(2))/dt;
            vz = (pose(3) - old_pose(3))/dt;
            
            ax = (pose(7) - old_pose(7))/dt;
            ay = (pose(8) - old_pose(8))/dt;
            az = (pose(9) - old_pose(9))/dt;
            
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
            
            %%%% Position SMC Control %%%%
            
            % X
            e1x = x - x_ref;
            dot_e1x = vx - dx_ref;
            e2x = vx - obj.J0*e1x*dt-obj.J1*e1x; % B를 넣어야할지? Desired velocity로 넣어야할지? 
            
            sx = obj.cx*e1x + e2x;
            
            Ux = obj.m*(dxdx_ref + obj.cx*(obj.J1 + obj.J0*e1x*dt - e2x) ...
                 - (obj.J1*dot_e1x + obj.J0*e1x) - obj.pix1*sign(sx) + obj.pix2*sx);

            % Y
            e1y = y - y_ref;
            dot_e1y = vy - dy_ref;
            e2y = vy - obj.J2*e1y*dt-obj.J3*e1y; % B를 넣어야할지? Desired velocity로 넣어야할지? 
            
            sy = obj.cy*e1y + e2y;
            
            Uy = obj.m*(dydy_ref + obj.cy*(obj.J3 + obj.J2*e1y*dt - e2y) ...
                 - (obj.J3*dot_e1y + obj.J2*e1y) - obj.piy1*sign(sy) + obj.piy2*sy);

            % Z
            e1z = z - z_ref;
            dot_e1z = vz - dz_ref;
            e2z = vz - obj.J4*e1z*dt-obj.J5*e1z; % B를 넣어야할지? Desired velocity로 넣어야할지? 
            
            sz = obj.cz*e1z + e2z;
            
            Uz = obj.m*(dzdz_ref + obj.cz*(obj.J5 + obj.J4*e1z*dt - e2z) ...
                 - (obj.J5*dot_e1z + obj.J4*e1z) - obj.piz1*sign(sz) + obj.piz2*sz);
            
            theta = atan((Ux*cos(yaw) + Uy*sin(yaw))/Uz);
            phi = atan(cos(theta)*(Ux*sin(yaw) - Uy*cos(yaw))/Uz);
            d_thrust = Uz/(cos(theta)*cos(phi));
            
            phi_dot = (phi - old_pose(4))/dt;
            theta_dot = (theta - old_pose(5))/dt;
            yaw_dot = (yaw - old_pose(6))/dt;
            
            old_pose = pose;
            
            d_att = [phi theta yaw phi_dot theta_dot yaw_dot d_thrust];
            
        end
        
        function input = attitude_control_run(obj, att, att_dot, att_d, att_d_dot, mode)
            global sigma
            persistent y_dot_pre pi
            if isempty(y_dot_pre)
                y_dot_pre = 0;
                pi = 0;
            end
            x = [att(1), att_dot(1), att(2), att_dot(2), att(3), att_dot(3)]';
            y = att(mode);
            y_d = att_d(mode);
            y_dot_d = att_d_dot(mode);
            y_dot = att_dot(mode);
            y_dotdot = (y_dot - y_dot_pre)/obj.dt;
            e1 = y_d - y;
            e1_dot = y_dot_d - y_dot;
            alpha = y_dot_d + obj.lambda1 * e1;
            e2 = y_dot - alpha ;
            Z = [x',y,y_dot,y_dotdot];
            fuzzy_basis = obj.fuzzy_basis_function(Z);
            neural_basis = obj.neural_basis_function(Z);
            basis = [fuzzy_basis, neural_basis];
            pi_dot  = (obj.gamma/(2*obj.tilde^2)) * e2^2 * (basis' * basis) - obj.gamma*sigma*pi;
            pi = pi + pi_dot*obj.dt;
            input = -obj.lambda2 * e2 - (1/(2*obj.tilde^2))*e2*pi * (basis' * basis);
            y_dot_pre = y_dot;
        end
        function fuzzy_output = fuzzy_membership(obj,x,num)
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
        function fuzzy_basis = fuzzy_basis_function(obj,x)
            membership_num = 5;
            pi_mu = 1;
            sum = 0;
            fuzzy_variable = x;
            pi_mu_array = zeros(1,membership_num);
            for j = 1:membership_num
                for eta = 1:size(fuzzy_variable,1)
                    mu=obj.fuzzy_membership(fuzzy_variable(eta),j);
                    pi_mu = pi_mu*mu;
                end
                pi_mu_array(j) = pi_mu;
                sum = sum+pi_mu;
            end
            fuzzy_basis = pi_mu_array/sum;
        end
        function neural_basis = neural_basis_function(obj,x)
            v = [-2 -1 0 1 2];
            global sigma
            sigma = 0.03;
            neural_basis_array = zeros(1,size(v,2));
            for j = 1:size(v,2)
                neural_basis_array(j) = exp(-1*norm(x-v(j))^2 / sigma);
            end
            neural_basis = neural_basis_array;
        end
    end
end

