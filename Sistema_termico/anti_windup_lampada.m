% organização do console
clc;
clear all;

% ARDUINO ----------------------------------------------------------------------------------------------------------------------------
a = arduino('COM7', 'Uno', 'Libraries', 'Adafruit/DHTxx');

% SENSOR ----------------------------------------------------------------------------------------------------------------------------
sensor_dht = addon(a, 'Adafruit/DHTxx', 'D5','DHT11');

% CONSTANTES PID ---------------------------------------------------------------------------------------------------------------------
SETPOINT = 50;
KC = 1; 

% Define os limites do PID
minPID = -10;
maxPID = 10;

% VARIAVEIS PID ----------------------------------------------------------------------------------------------------------------------
pid = 0;
last_pid = 0;

proporcional = 0; 
integrativo = 0; 
last_integrativo = 0; 
derivativo = 0; 

eaw = 0; 
last_eaw = 0; 

% measure = readTemperature(sensor_dht);
measure = 25; 
error = measure - SETPOINT; 
last_error = error; 
last2_error = error; 


% PWM ---------------------------------------------------------------------------------------------------------------------------------
low = 0;
high = 0;
duty = 0;
minDuty = 0;
maxDuty = 1;
signal_frequency = 2; 

% TEMPO DE AMOSTRAGEM ------------------------------------------------------------------------------------------------------------------
TS = 2;
TAW = 2;
ti = 1; 
td = 1;
current_time = tic;
elapse_time = toc(current_time);


% GRAFICO ------------------------------------------------------------------------------------------------------------------------------ 
figure(1); 
xlabel('Tempo');
ylabel("Temperatura");
title("Controlador de Temperatura");
temperature = animatedline('Color', 'b', 'LineWidth', 1.5); 
setpoint = animatedline('Color', 'r', 'LineStyle', '--', 'LineWidth', 1.5);
setpoint.DisplayName = 'setpoint';
temperature.DisplayName = 'temperatura'
legend('show');

figure(2); 
subplot(2,3,[1,2,3]); 
xlabel('Tempo'); 
ylabel('Temperatura'); 
title('Output PID'); 
pid_output = animatedline('Color', 'm', 'LineWidth', 1.5);

subplot(2,3,4); 
title('Proporcional'); 
p_output = animatedline('Color', 'g', 'LineWidth', 1.5);

subplot(2,3,5); 
title('Integrativo'); 
i_output = animatedline('Color', 'g', 'LineWidth', 1.5);

subplot(2,3,6); 
title('Derivativo'); 
d_output = animatedline('Color', 'g', 'LineWidth', 1.5);


while (true)
    measure = readTemperature(sensor_dht);
    
    elapse_time = toc(current_time);

    if (elapse_time > TS)
        
        error = measure - SETPOINT;

        proporcional = KC * error; 
        integrativo = last_integrativo + ((KC*TS)/ti)*last_error + (TS/TAW)*last_eaw; 
        derivativo = ((KC*td)/ts) * (error - last_error);
        pid = proporcional + integrativo + derivativo; 

        if pid <= minPID
            uf = minPID; 
            elseif pid >= maxPID
                uf = maxPID;
            else
                uf = pid;
        end 

        eaw = uf - pid; 
        last_eaw = eaw; 
        
        % duty = (pid - minPID) / (maxPID - minPID) * (maxDuty - minDuty) + minDuty;
        % duty = max(minDuty, min(maxDuty, duty));

        time_axis = round(double(current_time), 2);
        
        addpoints(temperature, time_axis, measure);
        addpoints(setpoint, time_axis, SETPOINT);
        addpoints(pid_output, time_axis, pid); 
        addpoints(p_output, time_axis, proporcional); 
        addpoints(i_output, time_axis, integrativo); 
        addpoints(d_output, time_axis, derivativo); 

        drawnow;

        current_time = tic;
        fprintf('Temp=%d | Ts=%d | Error=%d PID=%d|Duty=%d\n', measure,elapse_time,error,pid,duty);
        fprintf('PID = %d | P = %d | I = %d | D = %d \n', pid, proporcional, integrativo, derivativo);
    end

    generatePWM(a, signal_frequency, duty);

    last_pid = pid;
    last_error = error;
    last2_error = last_error;
end

function generatePWM(ino, frequency, dutyCycle)
    % Parâmetros:
    % - frequency: Frequência do sinal PWM em Hertz
    % - dutyCycle: Ciclo de trabalho em porcentagem (0 a 10)

    % Verifica se o ciclo de trabalho está dentro do intervalo permitido
    if dutyCycle < 0 || dutyCycle > 1
        error('O ciclo de trabalho deve estar entre 0 e 1.')
    end

    % Calcula o período com base na frequência
    period = 1 / frequency;

    % Calcula o tempo ativo (ligado) com base no ciclo de trabalho
    high = (dutyCycle / 1) * period;

    % Liga o sinal
    if high ~= 0
       writeDigitalPin(ino, 'D7', 1);
       pause(high);
    end

    % Desliga o sinal
    low = period - high;
    if low ~= 0
        writeDigitalPin(ino, 'D7', 0);
        pause(low);
    end
end