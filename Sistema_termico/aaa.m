clc; 
clear all;

% Load the data from the .mat file
y = load('yt_tentativa2.mat');  % Replace 'data.mat' with your file's name.
u = load('ut_tentativa2.mat')

figure(1)
plot(y.measurements);

figure(2);
plot(u.input)

% Customize the plot (add labels and a title)
xlabel('Data Point Index');
ylabel('Value');
title('Plot of Data');
