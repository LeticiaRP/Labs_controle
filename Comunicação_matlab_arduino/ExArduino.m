clc; 
clear; 

a = arduino("/dev/ttyUSB0", "Uno"); 

trimpot = []

disp("girar trimpot"); 
pause(1);

for i = 1:100
    trimpot(i) = readVoltage(a, 'A0');
    pause(0.1);
end

figure(1)
plot(trimpot, "Color", "b", 'LineWidth', 1.5)
title('Leitura analógica potênciometro')
