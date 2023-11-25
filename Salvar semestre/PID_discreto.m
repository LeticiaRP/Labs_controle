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

minPID = 0
maxPID = 0

pid_output = [0,0];

error = [0,0,0]; 


% --------- setup PWM 
time_on = 0;
time_off = 0;
frequency = 0;

duty_output = 0;
max_duty = 0;
min_duty = 0; 


% ---------- tempo de amostragem 
TS = 1/5; 
current_time = tic;
elapse_time = toc(current_time);


% ---------- gráficos
figure(1);
subplot(2,1,1);  
xlabel('Tempo');
ylabel("Temperatura");
title("Controlador de Temperatura");
temperature = animatedline('Color', 'b', 'LineWidth', 1.5); 
setpoint = animatedline('Color', 'r', 'LineStyle', '--', 'LineWidth', 1.5);
setpoint.DisplayName = 'setpoint';
temperature.DisplayName = 'temperatura'
legend('show');

subplot(2,1,2);  
xlabel('Tempo');
ylabel("Duty Cicle");
title("Duty Cicle Output");
duty_output = animatedline('Color', 'b', 'LineWidth', 1.5); 

figure(2); 
subplot(2,3,[1,2,3]); 
xlabel('Tempo'); 
ylabel('Acao de controle'); 
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

while(true)
    
end