% organização do console
clc;
clear all;

% ARDUINO ------------------------------------------------------------
a = arduino('COM3', 'Uno', 'Libraries', 'Adafruit/DHTxx');

% SENSOR ------------------------------------------------------------
sensor_dht = addon(a, 'Adafruit/DHTxx', 'D5','DHT11');

% Inicialização ------------------------------------------------------
sampleCount = 1;
measurements = [];
input = [];
time = [];
ts = 0.5;
current_time = tic;

% Condição Inicial ---------------------------------------------------
disp("Ligue a Lampada");
writeDigitalPin(a, 'D7', 0);
pause(5);

% Degrau -------------------------------------------------------------
writeDigitalPin(a, 'D7', 1);
input(sampleCount) = 1;
measurements(sampleCount) = readTemperature(sensor_dht);
time(sampleCount) = 0;

% Loop principal -----------------------------------------------------
while true
    elapse_time = toc(current_time);
    
    if (elapse_time > ts)
        % Atualize o contador e adicione a leitura e o tempo aos vetores
        sampleCount = sampleCount + 1;
        measurements(sampleCount) = readTemperature(sensor_dht);
        input(sampleCount) = 1;
        time(sampleCount) = toc(current_time);
        
        % Plot de ambas as variáveis no mesmo gráfico
        figure(1);
        plot(time, measurements, 'b', time, input, 'r');
        xlabel('Tempo (s)');
        ylabel('Temperatura');
        title('Temperatura');
        legend('Medida', 'Entrada de Degrau');
        
        drawnow;
        
        current_time = tic
    end
end
