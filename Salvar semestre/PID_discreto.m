% --------- organização console
warning off; 
clc; 
clear all; 

% ---------- setup hardware
arduino = arduino('COM7', 'Uno', 'Libraries', 'Adafruit/DHTxx');
sensor_dht = addon(a, 'Adafruit/DHTxx', 'D5','DHT11');

% ---------- variáveis PID
SETPOINT = 50; 

KP = 15;
KI = 1;
KD = 5;

min_PID = 0;
max_PID = 0;

pid_output = [0,0,0];
k = 3;

error = [0,0,0]; 

% --------- setup PWM 
time_on = 0;
time_off = 0;
frequency = 0;

duty_output = 0;
max_duty = 0;
min_duty = 0; 

% ---------- tempo de amostragem 
TS = 0.2; 
tic;        % inicia marcação de tempo 
t0 = tic;
elapse_time = toc;

% ---------- gráficos
figure(1);
subplot(2,1,1);  
xlabel('Tempo');
ylabel("Temperatura");
title("Controlador de Temperatura");
temperature = animatedline('Color', 'b', 'LineWidth', 1.5); 
setpoint = animatedline('Color', 'r', 'LineStyle', '--', 'LineWidth', 1.5);
setpoint.DisplayName = 'setpoint';
temperature.DisplayName = 'temperatura';
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
    sensor_measure = readTemperature(sensor_dht);

    if (elapse_time > TS) 

        error(k) = sensor_measure - SETPOINT; 

        pid_output(k) = pid_output(k-1) + [error(k) * (KP + KI * TS + KD / TS)] - [error(k-1) * (KP + (2 * KD) / TS)] + [error(k-2) * (KD / TS)];
    
        proporcional = KP*(error(k) - error(k-1)); 
        integrativo = KI*(error(k) * TS); 
        derivativo = KD*(error(k) - 2*[error(k-1)] + error(k-2))/TS;

        duty_output = (pid_output(k) - min_PID) / (max_PID - min_PID) * (max_duty - min_duty) + min_duty;
        duty_output = max(min_duty, min(max_duty, duty_output));

        time_axis = toc(t0);
        
        addpoints(temperature, time_axis, sensor_measure);
        addpoints(setpoint, time_axis, SETPOINT);
        addpoints(duty_output_, time_axis, duty_output);
        addpoints(pid_output_, time_axis, pid_output(k)); 
        addpoints(p_output, time_axis, proporcional); 
        addpoints(i_output, time_axis, integrativo); 
        addpoints(d_output, time_axis, derivativo); 

        drawnow;
        
        fprintf('Temp=%d | Ts=%d | Error=%d PID=%d|Duty=%d\n', sensor_measure, elapse_time, error(k), pid_output(k-1), duty_output);
        fprintf('PID = %d | P = %d | I = %d | D = %d \n', pid_output(k), proporcional, integrativo, derivativo);
        disp('no if')
        
        tic; % restart the timer
    end

    pid_output(k-1) = pid_output(k);
    error(k-1) = error(k);
    error(k-2) = error(k-1);
    disp('no while')
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
