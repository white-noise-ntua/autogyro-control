% Octave users
% pkg load control

%% User defined parameters 
m = 0.5; % mass (kg)
l = 0.1; % length (m)
g = 9.89; % gravitational acceleration (m / s^2)
omega_rpm = 1600; % speed of gyro (rpm)
omega = pi / 30 * omega_rpm;
% Inertial matrices
Ib = 0.01 * eye(3,3);
Ir = 0.002 * eye(3,3);

% F * h torque
F = 1;
h = 0.1;
tau = F * h;


%% Continuous Time Model

% Model definition
A = [0 1 0 0; 
    (-(Ir(3,3) + tau) / Ib(1,1) * omega^2) 0 0 (omega * (Ib(1,1) - Ir(3,3)) / Ib(1,1));
     0 0 0 1;
     0 ((Ir(3,3) - Ib(2,2)) / Ib(2,2) * omega) (-(Ir(3,3) + tau) / Ib(2,2) * omega^2) 0];

B = [0 0;
    (1 / Ib(1,1)) 0;
    0 0;
    0 (1 / Ib(2,2))];
 
C = eye(4, 4);
D = zeros(4, 2);

% State-space Model Definition
states = {'phi_x', 'phi_x_dot', 'phi_y', 'phi_y_dot'};
inputs = {'M_x' 'M_y'};

display('State-Space Model')
SS = ss(A, B, C, D, 'statename',states,'inputname',inputs,'outputname',states)

display('Transfer functions')
G = tf(SS)


% Extrema
x_1_max_deg = 5;
x_2_max_deg = 5;
x_1_max = 180 * x_1_max_deg / pi;
x_2_max = 180 * x_2_max_deg / pi;

M_x_max = 5e-2
M_y_max = M_x_max;

% Cheap control strategy
Q = C;
R = 4 * eye(2,2);
N = zeros(4,2);

[K, P, e] = lqr(A, B, Q, R, N);


% Controlled system

Ac = A - B * K;
Bc = zeros(4, 2);


SScl = ss(Ac, Bc, C, D,'statename',states,'inputname',inputs,'outputname',states);

t = 0 : 0.1 : 10;
r = zeros(2, length(t));
X0 = [0.01; 0; 0.01; 0];
[y,t,x]=lsim(SScl,r',t, X0);

% Feedback Control Law u = - K x
u_control = - (K * x')';

% Plots

% Outputs / States
figure;
[AX,~,~] = plotyy(t,y(:,1),t,y(:,2),'plot');
set(get(AX(1),'Ylabel'),'String','phi_x (rad)')
set(get(AX(2),'Ylabel'),'String','phi_y (rad)')
title('Step response')

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

[yd,t,xd]=lsim(SScl,r',t, X0);

% Feedback Control Law u = - K x
ud_control = - (Kd * xd')';

figure;
[AX,~,~] = plotyy(t,yd(:,1),t,yd(:,2),'stem');
set(get(AX(1),'Ylabel'),'String','phi_x (rad)')
set(get(AX(2),'Ylabel'),'String','phi_y (rad)')
title('Step response')

% Torques
figure;
[AX,~,~] = plotyy(t,ud_control(:,1),t,ud_control(:,2),'stem');
set(get(AX(1),'Ylabel'),'String','Torque M_x (Nm)')
set(get(AX(2),'Ylabel'),'String','Torque M_y (Nm)')
title('Inputs')
