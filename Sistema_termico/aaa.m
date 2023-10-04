clc; 

clear all;

arduino('/dev/ttyUSB0', 'Uno', 'Libraries', 'Adafruit/DHTxx');
readHumdity(a,Pin)
readTemperature(a,Pin)