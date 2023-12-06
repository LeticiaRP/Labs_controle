% clear the console 
clc; 
clear all; 
rosshutdown; 

pause(2)
 

%! ------------ ROS environment
% rosinit("192.168.1.102"); 
rosinit;

orientation_error_sub = rossubscriber('/controller/error','DataFormat','struct'); 
motion_direction_sub = rossubscriber('/controller/direction', 'DataFormat', 'struct'); 

cmd_vel_pub = rospublisher('/cmd_vel', 'geometry_msgs/Twist'); 

vel_msg = rosmessage(cmd_vel_pub); 

% --------- Limites de velocidade 
MAX_VEL_LINEAR = 2;
MIN_VEL_LINEAR = 0.3;



% ---------- Ganhos
KP = 1;
KI = 0;
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
Ts = 0.5; 
tic;        % inicia marcação de tempo 
t0 = tic;
elapse_time = toc;



%! ------------ PID Controller
while(true)

    elapse_time = toc;
    orientation_error = receive(orientation_error_sub,2);
    motion_direction = receive(motion_direction_sub, 2)

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
        % time_axis = toc(t0);
        
        tic; % restart the timer
    end

    derivative_filter(k-1) = derivative_filter(k);
    integrative(k-1) = integrative(k);
    error(k-1) = error(k);motion_direction
end




