%% Yaw Controller

%% System Definition

s = tf('s') ;
I3 = 1.53e-3;
% Yaw transfer function ddot psi = Mz / Ibz 
G = 1  / (s^2 * I3);

% PD Controller
Kp = 0.00324;
Kd = 0.0028;

% State-Space Gain Matrix
K = [Kp Kd];
csvwrite('K_yaw.csv', K);


% PD Controller 
C = Kp + Kd * s;

% Open-loop System
G_open = G * C;

G_cl = feedback(G * C, 1);

% Assert system is stable
rlocus(G_open); 

% Convert to State-Space Model
SScl = ss(G_cl);

% Simulate SS Model
t = 0 : 0.001 : 10;
r = zeros(1, length(t));
X0 = [0.1; 0;];
[y,t,x]=lsim(SScl,r',t, X0);

% Feedback Control Law u = - K x
u_control = - (K * x')';

%% Plot Results
figure;
hold on
[AX,~,~] = plotyy(t,x(:, 1),t,x(:, 2),'plot');

title('State Vectors')
set(get(AX(1),'Ylabel'),'String','phi_z / psi (rad)')
set(get(AX(2),'Ylabel'),'String','dot phi_z / dot psi (rad)')
xlabel('Time (s)')

hold off

figure;
hold on
plot(t, u_control);
title('Control Input z-Moment (Nm)')
ylabel('Mz (Nm)')
xlabel('Time (s)')
hold off
