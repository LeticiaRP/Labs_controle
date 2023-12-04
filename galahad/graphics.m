global angle
global setpoint

global duty_output
global pid_output_
global p_output
global i_output
global d_output

global time_axis

figure(1);
subplot(2,1,1);  
xlabel('Time');
ylabel("Pitch");
title("Self-Balance Robot");
angle = animatedline('Color', 'b', 'LineWidth', 1.5); 
setpoint = animatedline('Color', 'r', 'LineStyle', '--', 'LineWidth', 1.5);
setpoint.DisplayName = 'setpoint';
angle.DisplayName = 'current orientation';
legend('show');

subplot(2,1,2);  
xlabel('Time');
ylabel("Pitch");
title("Error");
error_ = animatedline('Color', 'b', 'LineWidth', 1.5); 

figure(2); 
subplot(2,3,[1,2,3]); 
xlabel('Time'); 
ylabel('Control atuation'); 
title('Output PID'); 
pid_output_ = animatedline('Color', 'm', 'LineWidth', 1.5);

subplot(2,3,4); 
title('Proporcional'); 
p_output = animatedline('Color', 'g', 'LineWidth', 1.5);

subplot(2,3,5); 
title('Integrative'); 
i_output = animatedline('Color', 'g', 'LineWidth', 1.5);

subplot(2,3,6); 
title('Derivative'); 
d_output = animatedline('Color', 'g', 'LineWidth', 1.5);

