 % ------- cleaning the console 
 clc; 
 clear all; 

% ------- hardware configuration 
esp = arduino('/dev/ttyUSB0', 'ESP32-WROOM-DevKitV1', 'Libraries','I2C');

motor1_IN1 = 'D25'
motor1_IN2 = 'D33' 
motor2_IN1 = 'D27'
motor2_IN2 = 'D26'

disp("movendo os motores para tras")

writePWMDutyCycle(esp, motor1_IN1, 0);
writePWMDutyCycle(esp, motor2_IN1, 0);

writePWMDutyCycle(esp, motor1_IN2, 1);
writePWMDutyCycle(esp, motor2_IN2, 1);


pause(5)


disp("movendo os motores para frente")

writePWMDutyCycle(esp, motor1_IN1, 1);
writePWMDutyCycle(esp, motor2_IN1, 1);

writePWMDutyCycle(esp, motor1_IN2, 0);
writePWMDutyCycle(esp, motor2_IN2, 0);
