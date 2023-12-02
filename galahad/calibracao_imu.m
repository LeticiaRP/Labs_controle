clc;
clear all;

% ARDUINO SETUP -------------------------------------------------------------
a = arduino('COM10', 'ESP32-WROOM-DevKitV1', 'Libraries','I2C');

bno = bno055(a, 'OperatingMode', 'ndof');

% Accelerometer calibration flag
accCalib = 0; 
% Gyroscope calibration flag
gyrCalib = 0; 
% Magnetometer calibration flag
magCalib = 0; 
fprintf('Calibrating the BNO055 sensor . . . \n');
while(prod([accCalib, gyrCalib, magCalib]) ~= 1)
    if strcmpi(bno.readCalibrationStatus.Accelerometer, "full") && isequal(accCalib, 0)
        accCalib = 1;
        fprintf('Accelerometer is calibrated! . . .\n');
    end
    if strcmpi(bno.readCalibrationStatus.Gyroscope, "full") && isequal(gyrCalib, 0)
        gyrCalib = 1;
        fprintf('Gyroscope is calibrated! . . .\n');
    end
    if(strcmpi(bno.readCalibrationStatus.Magnetometer, "full"))&& isequal(magCalib, 0)
        magCalib = 1;
        fprintf('Magnetometer is calibrated! . . .\n');
    end
end
fprintf('BNO055 sensor is fully calibrated!\n');

[readVal,timestamp]  = readOrientation(bno)
