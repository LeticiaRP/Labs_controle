[y, Fs] = audioread('francesco.wav');

% Projete o filtro Butterworth passa-baixa de 1ª ordem com frequência de corte de 0.2 (20% da frequência de Nyquist)
Wn = 0.04;
[b, a] = butter(2, Wn, 'low');

% Aplique o filtro ao sinal de áudio
y_filtro = filter(b, a, y);

% Plot do sinal de áudio original
subplot(2, 1, 1);
plot((0:length(y)-1) / Fs, y);
title('Sinal de Áudio Original');
xlabel('Tempo (s)');
ylabel('Amplitude');
grid on;

% Plot do sinal de áudio filtrado
subplot(2, 1, 2);
plot((0:length(y_filtro)-1) / Fs, y_filtro);
title('Sinal de Áudio Filtrado');
xlabel('Tempo (s)');
ylabel('Amplitude');
grid on;

% Ajuste o espaçamento entre os subplots
spacing = 0.05;
position = get(subplot(2, 1, 1), 'Position');
position(4) = position(4) - spacing;
set(subplot(2, 1, 1), 'Position', position);

% Reproduza o áudio original e o áudio filtrado
sound(y, Fs); % Áudio original
pause(length(y) / Fs); % Aguarde o áudio original terminar
sound(y_filtro, Fs); % Áudio filtrado




