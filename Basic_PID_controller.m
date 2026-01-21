clc; clear; close all;
s = tf('s');

% --- Gains & Motor Parameters ---
Rp2 = 5.660e3; Ri = 1.112e3; Rd2 = 0.5; R1 = 1e3; Ci = 1e-6; Cd = 0.1e-6;
Kp = Rp2 / R1; Ki = (Rp2 / R1) * (1/(Ri * Ci)); Kd = (Rp2 / R1) * (Rd2 * Cd);

Km_oz = 16.2; J_oz2 = 2.5; Ra = 11.5;
Km = Km_oz * 0.00706155; J = J_oz2 * 7.06155e-6;
Kb = 12/104.72; Kt = 12/104.72;
G = Km / (Ra*J*s + Kb*Kt);
H = 0.52 / 104.72;

start_rpm = -85;
target_rpm = 85;
delta_rpm = target_rpm - start_rpm; 

% Convert the RPM CHANGE into a voltage 
omega_change = delta_rpm * 2*pi/60;
u_step = H * omega_change; 

t = 0:1e-4:0.5;
u = u_step * ones(size(t));

controllers = {pid(Kp, 0, 0), pid(Kp, Ki, 0), pid(Kp, Ki, Kd)};
titles = {'P Control', 'PI Control', 'PID Control'};
colors = {'r', 'b', 'g'};

for i = 1:3
    % Standard Closed Loop
    CL = feedback(controllers{i}*G, H);
    
    % Simulate the CHANGE (starting from zero)
    y_change = lsim(CL, u, t);
    
    % Convert to RPM and SHIFT back to the -85 starting point
    y_rpm = (y_change * 60/(2*pi)) + start_rpm;
    
    
    figure;
    plot(t, y_rpm, colors{i}, 'LineWidth', 1.8);
    xlabel('Time (s)'); ylabel('Speed (RPM)');
    title([titles{i}, ' Classical Offset Response']);
    grid on; ylim([-100 110]);
end