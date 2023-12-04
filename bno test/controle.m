% --------- organização console
warning off; 
clc; 
clear all; 

% ---------- setup hardware
esp = arduino('/dev/ttyUSB0', 'ESP32-WROOM-DevKitV1', 'Libraries','I2C');
imu = bno055(esp, 'OperatingMode', 'ndof');

% ---------- variáveis PID legado
SETPOINT = 50; 

Kc = 1
Ti = 1
Td = 1

min_PID = 0;
max_PID = 0;

pid_output = 0;
k = 2;

error = [0,0]; 

% --------- derivative filter 
derivative_filter = [0,0];
cut_off_freq = 1  %rad/s 

% --------- anti-windup
integrative = [0,0]; 
tau = 1; 

min_output = 1; 
max_output = 1; 

limiter_output = 1; 

% --------- setup PWM 
time_on = 0;
time_off = 0;
frequency = 0;

duty_output = 0;
max_duty = 0;
min_duty = 0; 

% ---------- tempo de amostragem 
Ts = 0.2; 
tic;        % inicia marcação de tempo 
t0 = tic;
elapse_time = toc;

% ---------- gráficos
figure(1);
subplot(2,1,1);  
xlabel('Tempo');
ylabel("Orientação - pitch");
title("Controlador de Temperatura");
temperature = animatedline('Color', 'b', 'LineWidth', 1.5); 
setpoint = animatedline('Color', 'r', 'LineStyle', '--', 'LineWidth', 1.5);
setpoint.DisplayName = 'setpoint';
temperature.DisplayName = 'angulo';
legend('show');

subplot(2,1,2);  
xlabel('Tempo');
ylabel("Duty Cicle");
title("Duty Cicle Output");
duty_output_ = animatedline('Color', 'b', 'LineWidth', 1.5); 

figure(2); 
subplot(2,3,[1,2,3]); 
xlabel('Tempo'); 
ylabel('Acao de controle'); 
title('Output PID'); 
pid_output_ = animatedline('Color', 'm', 'LineWidth', 1.5);

subplot(2,3,4); 
title('Proporcional'); 
p_output = animatedline('Color', 'g', 'LineWidth', 1.5);

subplot(2,3,5); 
title('Integrativo'); 
i_output = animatedline('Color', 'g', 'LineWidth', 1.5);

subplot(2,3,6); 
title('Derivativo'); 
d_output = animatedline('Color', 'g', 'LineWidth', 1.5);

while(true)
    % verifica quanto tempo passou desde a função tic 
    elapse_time = toc;
    sensor_data = read(imu);
    sensor_measure = sensor_data.Orientation(end,2)

    if (elapse_time > Ts) 

        error(k) = sensor_measure - SETPOINT; 

        proporcional = Kc*error(k); 
        integrative(k) = integrative(k-1) + Kc*(Ts/Ti)*error(k-1); 

        %  -------- Anti-windup
        integrative(k) = integrative(k) + (Ts/tau)*(limiter_output - pid_output)
        

        % --------- filtro derivativo
        derivative_filter(k) = [derivative_filter(k-1) + cut_off_freq*error(k) - cut_off_freq*error(k-1)]/cut_off_freq*Ts; 
        derivative = Kc*(Td/Ts)*[error(k) - error(k-1)]

        pid_output = proporcional + integrative(k) + derivative*derivative_filter(k);
        
        % ------- Limitador
        if pid_output <= min_output
            limiter_output = min_output; 

            elseif pid_output >= max_output
                limiter_output = max_output

            else 
                limiter_output = pid_output
        end 

        duty_output = (pid_output - min_PID) / (max_PID - min_PID) * (max_duty - min_duty) + min_duty;
        duty_output = max(min_duty, min(max_duty, duty_output));

        time_axis = toc(t0);
        
        addpoints(temperature, time_axis, sensor_measure);
        addpoints(setpoint, time_axis, SETPOINT);
        addpoints(duty_output_, time_axis, duty_output);
        addpoints(pid_output_, time_axis, pid_output); 
        addpoints(p_output, time_axis, proporcional); 
        addpoints(i_output, time_axis, integrative(k)); 
        addpoints(d_output, time_axis, derivative); 

        drawnow;
        
        fprintf('Temp=%d | Ts=%d | Error=%d PID=%d|Duty=%d\n', sensor_measure, elapse_time, error(k), pid_output(k-1), duty_output);
        fprintf('PID = %d | P = %d | I = %d | D = %d \n', pid_output, proporcional, integrative(k), derivative);
        disp('no if')
        
        tic; % restart the timer
    end

    integrative(k-1) = integrative(k);
    error(k-1) = error(k);
end

function generatePWM(ino, frequency, dutyCycle)
    if dutyCycle < 0 || dutyCycle > 1
        error('O ciclo de trabalho deve estar entre 0 e 1.')
    end

    period = 1 / frequency;
    time_on = dutyCycle * period;

    % Liga o sinal
    if time_on > 0
       writeDigitalPin(ino, 'D7', 1);
       pause(time_on);
    end

    % Desliga o sinal
    time_off = period - time_on;
    if time_off > 0
        writeDigitalPin(ino, 'D7', 0);
        pause(time_off);
    end
end