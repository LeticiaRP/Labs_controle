% --------- organização console
warning off; 
clc; 
clear all; 

% ---------- setup hardware
arduino = arduino('COM7', 'Uno', 'Libraries', 'Adafruit/DHTxx');
sensor_dht = addon(a, 'Adafruit/DHTxx', 'D5','DHT11');

% ---------- gráficos
figure(1); 
xlabel('Tempo');
ylabel("Temperatura");
title("Controlador de Temperatura");
temperature = animatedline('Color', 'b', 'LineWidth', 1.5); 
temperature.DisplayName = 'temperatura';

disp("Ligue a Lampada");
%writeDigitalPin(a, 'D7', 0);
pause(5);

% ---------- tempo de amostragem 
Ts = 0.2; 
tic;        % inicia marcação de tempo 
t0 = tic;
elapse_time = toc;

% ---------- Degrau
writeDigitalPin(a, 'D7', 1);

while(true)
    % verifica quanto tempo passou desde a função tic 
    elapse_time = toc;
    sensor_measure = readTemperature(sensor_dht);

    if (elapse_time > Ts) 

        time_axis = toc(t0);
        
        addpoints(temperature, time_axis, sensor_measure);
    
        drawnow;
        
        tic; % restart the timer
    end

    
end
