% organização do console
clc;
clear all;

% dicas:
% 1 - colocar ; no final do comando faz com que o output NAO apareça no console
% 2 - comandos SEM ; no final, faz com que APAREÇA o output no console

% criar objeto 'arduino' no matlab e inicia a comunicação serial com o software
ino = arduino('COM7', 'Uno', 'Libraries', 'Adafruit/DHTxx')

% criar objeto sensor
sensor_dht = addon(a, 'Adafruit/DHTxx', 'D3','DHT11')

SETPOINT = 50;

KP = 1;
KI = 0;
KD = 0;
low = 0;
pid = 0;
last_pid = 0;

error = 0;
last_error = 0;
last2_error = 0;

measure = 0;
last_sample = 25;

duty = 0;

ts = 0.5;
current_time = tic;
elapse_time = toc(current_time);

% Configuração do gráfico em tempo real
% figure;
% ax = axes;
% line = animatedline(ax, 'Color', 'b'); % Configura a cor da linha

% xlabel('Tempo');
% ylabel('Medida');
% title('Leitura em Tempo Real');

% Defina os limites dos eixos, se necessário
% ylim([ymin, ymax]);

writeDigitalPin(ino, 'D13', 0)

while (true)
    measure = readTemperature(sensor_dht);
    
    if (last_sample - measure) > 5 
        measure = last_sample; 
    end 
    
    elapse_time = toc(current_time);
    if (elapse_time > ts)
        
        error = measure - SETPOINT

        pid = last_pid + error * (KP + KI * ts + KD / ts) - last_error * (KP + (2 * KD) / 2) + last2_error * (KD / ts);

        duty = abs(1 - ((pid - (-20)) * (0 - 1) / (35 - (-20)) + 1));
        
        if duty > 1
            duty = 1
        end 
        
        if duty < 0
            duty = 0
        end
        
        measure = measure
        
        % % Atualize o gráfico em tempo real
        % addpoints(line, toc(current_time), measure);
        % drawnow limitrate; % Limita a taxa de atualização para manter um gráfico mais suave

        % COLOCAR O PWM MANUALMENTE
        current_time = tic;
    end

    generatePWM(ino, 0.5, duty);

    last_pid = pid;
    last_error = error;
    last2_error = last_error;
    last_sample = measure

end

function generatePWM(ino, frequency, dutyCycle)
    % Parâmetros:
    % - frequency: Frequência do sinal PWM em Hertz
    % - dutyCycle: Ciclo de trabalho em porcentagem (0 a 100)

    % Verifica se o ciclo de trabalho está dentro do intervalo permitido
    if dutyCycle < 0 || dutyCycle > 1
        error('O ciclo de trabalho deve estar entre 0 e 1.')
    end

    % Calcula o período com base na frequência
    period = 1 / frequency;

    % Calcula o tempo ativo (ligado) com base no ciclo de trabalho
    activeTime = (dutyCycle / 1) * period;

    % Liga o sina
    writeDigitalPin(ino, 'D13', 1);
    pause(activeTime);

    % Desliga o sinal
    writeDigitalPin(ino, 'D13', 0);
    low = period - activeTime;
    % Espera por um novo ciclo
    pause(low);
end