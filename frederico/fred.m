% clear the console 
clc; 
clear all; 
rosshutdown; 

pause(2)
 

%! ------------ ROS environment
rosinit("192.168.1.102"); 
% rosinit;

orientation_error_sub = rossubscriber('/controller/error','DataFormat','struct'); 
setpoint_sub = rossubscriber('/controller/setpoint', 'DataFormat', 'struct'); 
motion_direction_sub = rossubscriber('/controller/direction', 'DataFormat', 'struct'); 
odom_sub = rossubscriber('/odom', 'DataFormat', 'struct');
goal_sub = rossubscriber('/goal_manager/goal/current', 'DataFormat', 'struct');
navigation_sub = rossubscriber('/navigation/on', 'DataFormat', 'struct')


cmd_vel_pub = rospublisher('/cmd_vel', 'geometry_msgs/Twist'); 

vel_msg = rosmessage(cmd_vel_pub); 

% --------- Limites de velocidade 
MAX_VEL_LINEAR = 2;
MIN_VEL_LINEAR = 0.3;



% ---------- Ganhos
KP = 20;
KI = 5;
KD = 0;


% --------- Limites da resposta do controlador 
min_output = -20; 
max_output = 20; 


% --------- Saída PID
pid_output = 0;


% --------- Saturação PID
limiter_output = 0; 


% --------- Regressão 
k = 2;


% ---------- Regressão erro
error = [0,0]; 



% --------- Anti-windup
integrative = [0,0]; 
tau = 1; 


% --------- Filtro derivativo
derivative_filter = [0,0]; 
cut_off_freq = 1;       % em rad


% ---------- tempo de amostragem 
Ts = 1/10; 
tic;        % inicia marcação de tempo 
t0 = tic;
elapse_time = toc;



% --------- plot setup
% figure(1); 
% title('Robot Pose and Goal');
% xlabel('X Position (m)');
% ylabel('Y Position (m)');
% grid on;
% hold on;

% ---------- gráficos
figure(1);
subplot(2,1,1);  
title('Robot Pose and Goal');
xlabel('Y Position (m)');
ylabel('X Position (m)');
robot_position = animatedline('Color', 'b', 'LineWidth', 1.5); 
goal_position = animatedline('Color', 'm', 'LineStyle', 'none', 'Marker', 'pentagram');
robot_position.DisplayName = 'Robot Pose';
goal_position.DisplayName = 'Goal Pose';
legend('show');
grid on;

subplot(2,1,2);  
xlabel('Time');
ylabel("Yaw angle (rad)");
title('Orientation Control');
robot_orientation = animatedline('Color', '#4DBEEE', 'LineWidth', 1.5); 
setpoint_angle = animatedline('Color', '#D95319', 'LineStyle', '--', 'LineWidth', 1.5);
setpoint_angle.DisplayName = 'Setpoint';
robot_orientation.DisplayName = 'Robot Heading';
legend('show');

figure(2); 
subplot(2,3,[1,2,3]); 
xlabel('Time'); 
ylabel('Response'); 
title('Output PID'); 
pid_output_ = animatedline('Color', '#7E2F8E', 'LineWidth', 1.5);

subplot(2,3,4); 
title('Proporcional'); 
p_output = animatedline('Color', '#EDB120', 'LineWidth', 1.5);

subplot(2,3,5); 
title('Integrative'); 
i_output = animatedline('Color', '#EDB120', 'LineWidth', 1.5);

subplot(2,3,6); 
title('Derivative'); 
d_output = animatedline('Color', '#EDB120', 'LineWidth', 1.5);



%! ------------ PID Controller
while(true)

    navigation_flag = receive(navigation_sub);

    if (navigation_flag.data == true)

        elapse_time = toc;
        orientation_error = receive(orientation_error_sub,2);
        motion_direction = receive(motion_direction_sub, 2); 

        setpoint = receive(setpoint_sub, 2); 

        if (elapse_time > Ts) 

            % error(k) = sensor_measure - SETPOINT; 

            error(k) = orientation_error.data;

            proporcional = KP*error(k); 

            %!  -------- Anti-windup
            integrative(k) = integrative(k-1) + KI*(Ts)*error(k-1) + (Ts/tau)*(limiter_output - pid_output); 
            

            %! --------- filtro derivativo
            derivative_filter(k) = [derivative_filter(k-1) + cut_off_freq*error(k) - cut_off_freq*error(k-1)]/cut_off_freq*Ts; 
            derivative = (KD/Ts)*[error(k) - error(k-1)]; 

            pid_output = proporcional + integrative(k) + derivative*derivative_filter(k);
            
            % ------- Limitador
            if pid_output <= min_output
                limiter_output = min_output; 

                elseif pid_output >= max_output
                    limiter_output = max_output;

                else 
                    limiter_output = pid_output;
            end 

            fprintf('PID output = %d |  Limiter_output = %d\n', pid_output, limiter_output);

            vel_msg.Linear.X = ((1-abs(orientation_error.data)/pi)*(MAX_VEL_LINEAR - MIN_VEL_LINEAR) + MIN_VEL_LINEAR) * motion_direction.data; 
            vel_msg.Angular.Z = limiter_output; 

            send(cmd_vel_pub, vel_msg);
            time_axis = toc(t0);
            
            
            % -------- Dados odometria
            odom_msg = receive(odom_sub);
            
            robot_pose_x = odom_msg.pose.pose.position.x;
            robot_pose_y = odom_msg.pose.pose.position.y;
            robot_orientation_ = odom_msg.pose.pose.orientation.z; 


            % -------- Dados goal
            goal_msg = receive(goal_sub);
            
            goal_pose_x = goal_msg.pose.position.x;
            goal_pose_y = goal_msg.pose.position.y;


            addpoints(robot_position, robot_pose_y, robot_pose_x);
            addpoints(goal_position, goal_pose_y, goal_pose_x);
            addpoints(robot_orientation, time_axis, robot_orientation_); 
            addpoints(setpoint_angle, time_axis, setpoint.data); 
            addpoints(pid_output_, time_axis, limiter_output); 
            addpoints(p_output, time_axis, proporcional); 
            addpoints(i_output, time_axis, integrative(k)); 
            addpoints(d_output, time_axis, derivative); 

            drawnow; 

            tic; % restart the timer
        end

        derivative_filter(k-1) = derivative_filter(k);
        integrative(k-1) = integrative(k);
        error(k-1) = error(k);
    end

end