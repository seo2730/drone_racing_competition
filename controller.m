classdef controller < handle
    properties
        % System model or other things
        
        % Quadrotor specs
        Ixx = 8.1*10^(-3);  % Quadrotor moment of inertia around X axis  (X축의 관성 모멘트)
        Iyy = 8.1*10^(-3);  % Quadrotor moment of inertia around Y axis  (Y축의 관성 모멘트)
        Izz = 14.2*10^(-3); % Quadrotor moment of inertia around Z axis (Z축의 관성 모멘트)
        m = 1.1;            % Mass of the Quadrotor in Kg(드론 무게, 단위 : kg)
        g = 9.81;           % Gravitational acceleration (중력 가속도)
        
        % Sliding control
        cx = 1; J0 = 1; J1 = 1; pix1 = 1; pix2 = 1; % design parameter in x
        cy = 1; J2 = 1; J3 = 1; piy1 = 1; piy2 = 1; % design parameter in y
        cz = 1; J4 = 1; J5 = 1; piz1 = 1; piz2 = 1; % design parameter in z
        
    end
    
     methods
        function model = controller() % 생성자 
            % System model or other things
            
        end
        
        function input = control_run(model,pose, pose_d, velocity_d, accel_d)
            %d_att = model.position_controller(pose, pose_d, velocity_d, accel_d);
           input = model.position_controller(pose, pose_d, velocity_d, accel_d);
            %input = model.attitude_controller(d_att);
        end
        
        function d_att = position_controller(model, pose, pose_d, velocity_d, accel_d)
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
            
            ax = (pose(4) - old_pose(4))/dt;
            ay = (pose(5) - old_pose(5))/dt;
            az = (pose(6) - old_pose(6))/dt;

            old_pose = pose;
            
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
            e2x = vx - dx_ref; % B를 넣어야할지? Desired velocity로 넣어야할지? 
            
            sx = model.cx*e1x + e2x;
            
            Ux = model.m*(dxdx_ref + model.cx*(model.J1 + model.J0*e1x*dt - e2x) ...
                 - (model.J1*dot_e1x + model.J0*e1x + model.pix1*sign(sx) + model.pix2*sx));

            % Y
            e1y = y - y_ref;
            dot_e1y = vy - dy_ref;
            e2y = vy - dy_ref; % B를 넣어야할지? Desired velocity로 넣어야할지? 
            
            sy = model.cy*e1y + e2y;
            
            Uy = model.m*(dydy_ref + model.cy*(model.J3 + model.J2*e1y*dt - e2y) ...
                 - (model.J3*dot_e1y + model.J2*e1y + model.piy1*sign(sy) + model.piy2*sy));

            % Z
            e1z = z - z_ref;
            dot_e1z = vz - dz_ref;
            e2z = vz - dz_ref; % B를 넣어야할지? Desired velocity로 넣어야할지? 
            
            sz = model.cz*e1z + e2z;
            
            Uz = model.m*(dzdz_ref + model.cz*(model.J5 + model.J4*e1z*dt - e2z) ...
                 - (model.J5*dot_e1z + model.J4*e1z + model.piz1*sign(sz) + model.piz2*sz));

             d_thrust = 14;
            
            d_att = [Ux Uy Uz d_thrust];
        end
        
        function commands = attitude_controller(model,d_att)
        
            commands = [roll_ref pitch_ref yaw_ref thrust];
        end
        
    end
end