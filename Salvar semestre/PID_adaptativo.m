% organização do console
clc;
clear all;

% ARDUINO ----------------------------------------------------------------------------------------------------------------------------
a = arduino('COM3', 'Uno', 'Libraries', 'Adafruit/DHTxx');

% SENSOR ----------------------------------------------------------------------------------------------------------------------------
sensor_dht = addon(a, 'Adafruit/DHTxx', 'D5','DHT11');

% condição inicial 
y_h(1) =readTemperature(sensor_dht);

lampda = 1;
indice = 2; 

% regressores
y_t = [ ];
y_h = [ ];
u_t = [ ]; 

% condição inicial 
y_t(1) = 25;
y_h(1) = 0;
u_t(1) = 1;  

% incerteza
P = 100000 * eye(2,2); 

% estimador 
est = 0;

% ganho 
K = [0; 0]

% theta 
theta = [0; 0]; 

% GRAFICO ------------------------------------------------------------------------------------------------------------------------------ 
figure(1); 
subplot(2,1,1); 
xlabel('Tempo');
ylabel("Temperatura");
title("Temperatura estimada");
estimativa = animatedline('Color', 'b', 'LineWidth', 1.5); 

subplot(2,1,2);  
xlabel('Tempo');
ylabel("Temperatura");
title("Temperatura medida");
temperatura = animatedline('Color', 'm', 'LineWidth', 1.5); 

% Condição Inicial ---------------------------------------------------
disp("Ligue a Lampada");
writeDigitalPin(a, 'D7', 0);
pause(5);

y_h(1) = readTemperature(sensor_dht);

current_time = tic;

% Degrau -------------------------------------------------------------
writeDigitalPin(a, 'D7', 1);

while (true)

        y_t(indice) = readTemperature(sensor_dht);
        u_t(indice) = readVoltage(a,"A2");

        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        %%%%%%%%%%%%%%%%%% matriz de regressores %%%%%%%%%%%%%%%%%%%%%%%
        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

        % matriz psi 
        psi = [y_t(indice - 1) u_t(indice - 1)];
        
        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        %%%%%%%%%%%%%%%%%%%%% calculo theta %%%%%%%%%%%%%%%%%%%%%%%%%%%%
        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        
        K = (P * psi') / (lampda + (psi * P * psi')); 

        est = y_t(indice) - y_h(indice - 1)

        theta = theta + K * est;  

        P = P - K*(1 + psi * P * psi') * K'; 
        
        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        %%%%%%%%%%%%%%%%%%%%%%% modelo ARX discreto %%%%%%%%%%%%%%%%%%%%
        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        
        y_h(indice) = theta(1) * y_t(indice-1) + theta(2) * u_t(indice-1);

        addpoints(estimativa, toc(current_time), y_h(indice));
        addpoints(temperatura, toc(current_time), y_t(indice));
        drawnow;

        indice = indice + 1; 
end