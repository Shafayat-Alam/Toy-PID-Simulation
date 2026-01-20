clc; clear; close all;
s = tf('s');

Rp2 = 5.660e3;      
Ri  = 1.112e3;      
Rd2 = 0.5;          
R1 = 1e3;           
Ci = 1e-6;          
Cd = 0.1e-6;        

Kp = Rp2 / R1;
Ki = (Rp2 / R1) * (1/(Ri * Ci));
Kd = (Rp2 / R1) * (Rd2 * Cd);

Km_oz = 16.2;                    
J_oz2 = 2.5;                    
Ra = 11.5;                      
Km = Km_oz * 0.00706155;        
J  = J_oz2 * 7.06155e-6;        
Kb = 12/104.72;                 
Kt = 12/104.72;                 
G = Km / (Ra*J*s + Kb*Kt);
H = 0.52 / 104.72;

rpm_desired = 85;
omega_des = rpm_desired * 2*pi/60;
r_voltage = H * omega_des;

t = 0:1e-4:0.5;
u = r_voltage * ones(size(t));
header = {'time_s','input_voltage','output_rpm'};

% --- start at -85 RPM ---
omega_init = -85 * 2*pi/60;   

%% ===== P Controller =====
C_P = pid(Kp, 0, 0);
CL_P = feedback(C_P*G, H);
yP = lsim(CL_P, u, t, omega_init);   
yP_rpm = yP * 60/(2*pi);

dataP = [t(:), u(:), yP_rpm(:)];
fid = fopen('p_simulation.csv','w');
fprintf(fid, '%s,%s,%s\n', header{:});
fclose(fid);
writematrix(dataP,'p_simulation.csv','WriteMode','append');

figure;
plot(t, yP_rpm, 'r', 'LineWidth', 1.8);
xlabel('Time (s)'); ylabel('Speed (RPM)');
title('Proportional (P) Control Response');
grid on; ylim([-100 110]);

%% ===== PI Controller =====
C_PI = pid(Kp, Ki, 0);
CL_PI = feedback(C_PI*G, H);
yPI = lsim(CL_PI, u, t, omega_init);
yPI_rpm = yPI * 60/(2*pi);

dataPI = [t(:), u(:), yPI_rpm(:)];
fid = fopen('pi_simulation.csv','w');
fprintf(fid, '%s,%s,%s\n', header{:});
fclose(fid);
writematrix(dataPI,'pi_simulation.csv','WriteMode','append');

figure;
plot(t, yPI_rpm, 'b', 'LineWidth', 1.8);
xlabel('Time (s)'); ylabel('Speed (RPM)');
title('Proportional–Integral (PI) Control Response');
grid on; ylim([-100 110]);

%% ===== PID Controller =====
C_PID = pid(Kp, Ki, Kd);
CL_PID = feedback(C_PID*G, H);
yPID = lsim(CL_PID, u, t, omega_init);
yPID_rpm = yPID * 60/(2*pi);

dataPID = [t(:), u(:), yPID_rpm(:)];
fid = fopen('pid_simulation.csv','w');
fprintf(fid, '%s,%s,%s\n', header{:});
fclose(fid);
writematrix(dataPID,'pid_simulation.csv','WriteMode','append');

figure;
plot(t, yPID_rpm, 'g', 'LineWidth', 1.8);
xlabel('Time (s)'); ylabel('Speed (RPM)');
title('Proportional–Integral–Derivative (PID) Control Response');
grid on; ylim([-100 110]);
