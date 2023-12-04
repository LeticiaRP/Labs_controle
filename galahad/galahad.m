% ------- cleaning the console 
clc; 
clear all; 



% ------- hardware configuration 
esp = arduino('/dev/ttyUSB0', 'ESP32-WROOM-DevKitV1', 'Libraries','I2C');
imu = bno055(esp, 'OperatingMode', 'ndof', 'ReadMode','latest');

motor1_IN1 = 'D25';
motor1_IN2 = 'D33';
motor2_IN1 = 'D27';
motor2_IN2 = 'D26';



% ------- ROS2 connection 
angle_node = ros2node("/setpoint_angle");
pause(5);                                                 % PRECISA DESSE DELAY
angle_sub = ros2subscriber(angle_node, "/setpoint/angle");

[IN_BALANCE_SETPOINT,status,statustext] = receive(angle_sub,10);



% ------ calibration of the bno sensor
sensor_calibration 



% ------ make the graphics setups
graphics



% ------ inicialize the controller variables 
% controller_variables
KP = 1;
KI = 0; 
KD = 0; 

Ts = 0.2;
tau = 1; 
t0 = tic; 

cut_off_freq = 1; % in rad

k = 2;

error = [0,0];
integrative = [0,0];
derivative_filter = [0,0]; 

limiter_output = 0; 
pid_output = 0; 

min_output = -1;
max_output = 1;



while(statustext == 'success')  

    % receive the topic message
    [IN_BALANCE_SETPOINT,status,statustext] = receive(angle_sub,10);

    elapse_time = toc; 
    imu_data = read(imu);

    robot_pitch = imu_data.Orientation(end, 2);
    
    if (elapse_time >= Ts)

        error(k) = robot_pitch - IN_BALANCE_SETPOINT.data; 

        %! -------------------------- PROPORCIONAL
        proporcional = error(k) * KP; 


        %! -------------------------- INTEGRATIVE WITH ANTI-WINDUP
        integrative(k) = integrative(k-1) + KI*Ts*error(k-1) + (Ts/tau)*(limiter_output - pid_output); 


        %! -------------------------- DERIVATIVE WITH LOW-PASS FILTER
        derivative = (KD/Ts)*[error(k) - error(k-1)];
        low_pass_filter(k) = [derivative_filter(k-1) + cut_off_freq*error(k) - cut_off_freq*error(k-1)]/cut_off_freq*Ts; 


        %! -------------------------- PID OUTPUT
        pid_output = proporcional + integrative(k) + derivative*low_pass_filter(k); 


        %! -------------------------- LIMITER
        if pid_output <= min_output
            limiter_output = min_output; 

            elseif pid_output >= max_output
                limiter_output = max_output;

            else 
                limiter_output = pid_output;
        end 

        %! -------------------------- MOTORS ACTUATION
        if limiter_output < 0
            writePWMDutyCycle(esp, motor1_IN1, 0);
            writePWMDutyCycle(esp, motor2_IN1, 0);

            writePWMDutyCycle(esp, motor1_IN2, abs(limiter_output));
            writePWMDutyCycle(esp, motor2_IN2, abs(limiter_output));
        else
            writePWMDutyCycle(esp, motor1_IN1, limiter_output);
            writePWMDutyCycle(esp, motor2_IN1, limiter_output);

            writePWMDutyCycle(esp, motor1_IN2, 0);
            writePWMDutyCycle(esp, motor2_IN2, 0);
        end 

        

        time_axis = toc(t0);
        
        addpoints(angle, time_axis, robot_pitch);
        addpoints(setpoint, time_axis, IN_BALANCE_SETPOINT.data);
        addpoints(error_, time_axis, error(k));
        addpoints(pid_output_, time_axis, pid_output); 
        addpoints(p_output, time_axis, proporcional); 
        addpoints(i_output, time_axis, integrative(k)); 
        addpoints(d_output, time_axis, derivative); 
    
        drawnow;
    


    end

    error(k-1) = error(k)

    integrative(k-1) = integrative(k)

    low_pass_filter(k-1) = low_pass_filter(k)

end
