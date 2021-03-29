classdef controller < handle
    properties
        % System model or other things

    end
    
    methods
        function model = controller() % 생성자 
            % System model or other things
            
        end
        
        function input = control_run(model,pose, pose_d, velocity_d)
            input = model.position_controller(pose, pose_d, velocity_d);
        end
        
        function commands = position_controller(model,pose, pose_d, velocity_d)
            global dt initial_state;
            persistent old_pose; % stores previous pose
            
            if isempty(old_pose)
                old_pose = initial_state(1:6);
            end
            
            Kp_x = 1;
            Kd_x = 2;

            Kp_y = 1;
            Kd_y = 2;

            Kp_z = 50;
            Kd_z = 10;
            
            x = pose(1);
            y = pose(2);
            z = pose(3);
            yaw = pose(6);

            vx = (pose(1) - old_pose(1))/dt;
            vy = (pose(2) - old_pose(2))/dt;
            vz = (pose(3) - old_pose(3))/dt;

            old_pose = pose;
            
            x_ref = pose_d(1);
            y_ref = pose_d(2);
            z_ref = pose_d(3);
            yaw_ref = pose_d(4);

            dx_ref = velocity_d(1);
            dy_ref = velocity_d(2);
            dz_ref = velocity_d(3);

            e_x = cos(yaw)*(x_ref - x) - sin(yaw)*(y_ref - y);
            e_dx = cos(yaw)*(dx_ref - vx) - sin(yaw)*(dy_ref - vy);

            e_y = sin(yaw)*(x_ref - x) + cos(yaw)*(y_ref - y);
            e_dy = sin(yaw)*(dx_ref - vx) + cos(yaw)*(dy_ref - vy);

            e_z = z_ref - z;
            e_dz = dz_ref - vz;
            
            roll_ref = Kp_y*e_y + Kd_y*e_dy; %
            pitch_ref = -Kp_x*e_x - Kd_x*e_dx; %
            thrust = 1.1*9.81 + ...     % hower thrust = mass*gravity
                Kp_z*e_z + Kd_z*e_dz;   % total thrust on the body along z-axis

            roll_ref = max([min([roll_ref pi/2]) -pi/2]); %
            pitch_ref = max([min([pitch_ref pi/2]) -pi/2]); %

            commands = [roll_ref pitch_ref yaw_ref thrust];
        end
    end
end