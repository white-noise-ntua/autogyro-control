%% User defined parameters 
m = 5; % mass (kg)
l = 0.1; % length (m)
g = 9.89; % gravitational acceleration (m / s^2)
omega_rpm = 1000; % speed of gyro (rpm)
omega = pi / 30 * omega_rpm;
I_R = 0.05; % moment of inertia about yaw axis

% Auto-computed parameters
L = m * g * l;
M = 1 / (I_R * omega);

%% Continuous Time Model

% Model definition
A = [0 L * M; -L * M 0];
B = [0 -M; M 0];
C = [1 0; 0 1];
D = [0 0; 0 0];

% State-space Model Definition
states = {'phi_x' 'phi_y'};
inputs = {'M_x' 'M_y'};

display('State-Space Model')
SS = ss(A, B, C, D, 'statename',states,'inputname',inputs,'outputname',states)

display('Transfer functions')
G = tf(SS)


% Extrema
x_1_max_deg = 20;
x_2_max_deg = 20;
x_1_max = 180 * x_1_max_deg / pi;
x_2_max = 180 * x_2_max_deg / pi;

M_x_max = 10;
M_y_max = M_x_max;

% Cheap control strategy
Q = [100 25 ; 25 100];
R = [1 0; 0 1];
N = [0 0; 0 0];

[K, P, e] = lqr(A, B, Q, R, N);


% Controlled system

Ac = A - B * K;
Bc = [0 0; 0 0];


SScl = ss(Ac, Bc, C, D,'statename',states,'inputname',inputs,'outputname',states);

t = 0 : 0.1 : 10;
r = zeros(2, length(t));
X0 = [0.1; 0.1];
[y,t,x]=lsim(SScl,r,t, X0);

% Feedback Control Law u = - K x
u_control = - (K * x')';

% Plots

% Outputs / States
figure;
[AX,~,~] = plotyy(t,y(:,1),t,y(:,2),'plot');
set(get(AX(1),'Ylabel'),'String','phi_x (rad)')
set(get(AX(2),'Ylabel'),'String','phi_y (rad)')
title('Step Response with LQR Control')

% Torques
figure;
[AX,~,~] = plotyy(t,u_control(:,1),t,u_control(:,2),'plot');
set(get(AX(1),'Ylabel'),'String','Torque M_x (Nm)')
set(get(AX(2),'Ylabel'),'String','Torque M_y (Nm)')
title('Inputs')

%% Discretization
T = 1 / 100; % Sampling Period
SSd = c2d(SS, T, 'zoh');
Gz = tf(SSd);

[Kd, Pd, ed] = lqr(SSd.A, SSd.B, Q, R, N);

Acd = A - B * K;
SScld = ss(Ac, Bc, C, D,'statename',states,'inputname',inputs,'outputname',states);

[yd,t,xd]=lsim(SScl,r,t, X0);

% Feedback Control Law u = - K x
ud_control = - (Kd * xd')';

figure;
[AX,~,~] = plotyy(t,yd(:,1),t,yd(:,2),'stem');
set(get(AX(1),'Ylabel'),'String','phi_x (rad)')
set(get(AX(2),'Ylabel'),'String','phi_y (rad)')
title('Step Response with LQR Control (Discrete Time - Zero-order Hold)')

% Torques
figure;
[AX,~,~] = plotyy(t,ud_control(:,1),t,ud_control(:,2),'stem');
set(get(AX(1),'Ylabel'),'String','Torque M_x (Nm)')
set(get(AX(2),'Ylabel'),'String','Torque M_y (Nm)')
title('Inputs (Discrete Time - Zero-order Hold)')
