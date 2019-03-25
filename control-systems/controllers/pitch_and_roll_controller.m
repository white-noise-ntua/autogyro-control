% Octave users
% pkg load control

%% User defined parameters 
m = 75e-3;
r = 0.35;
M = 0.33;
mu = (M * m) / (M + m);
R = 0.05;
H = 0.3;
h = 0.15;
rho = h * m / (M + m);
rho_h = h * M / (M + m);
Ix = 1 / 4 * m * r^2 + 1 / 12 * M * (3 * R^2 + H^2) + M * rho_h ^ 2 + m * rho^2;
Iy = Ix;
Iz = 1 / 2 * m * r^2;


I1 = Ix;
I3 = Iz;
I2 = Iz - Iy;

g = 9.89; % gravitational acceleration (m / s^2)
omega_rpm = 500; % speed of gyro (rpm)
omega = pi / 30 * omega_rpm;
% Inertial matrices by Fusion
% I1 = 4.75e-3;
% I2 = -3.24e-3;
% I3 = 1.53e-3;


alpha = I2 / I1 * omega;

%% Continuous Time Model

% Model definition
A = [ 0 1 0 0; 
      0 0 0 -alpha;  
      0 0 0 1;
      0 alpha 0 0];

B = [0 0;
     1 0;
     0 0;
     0 1];
 
C = eye(4, 4);
D = zeros(4, 2);

% State-space Model Definition
states = {'phi_x', 'phi_x_dot', 'phi_y', 'phi_y_dot'};
inputs = {'M_x' 'M_y'};

display('State-Space Model')
SS = ss(A, B, C, D, 'statename',states,'inputname',inputs,'outputname',states)

display('Transfer functions')
G = tf(SS)

M_x_max = 2e-3;
M_y_max = M_x_max;
phi_x_max = 10 * pi / 180;

phi_y_max = phi_x_max;

% Cheap control strategy
Q = 3 * eye(4,4);
R =  eye(2,2);
N = zeros(4,2);

[K, P, e] = lqr(A, B, Q, R, N);


% Controlled system

Ac = A - B * K;
Bc = zeros(4, 2);


SScl = ss(Ac, Bc, C, D,'statename',states,'inputname',inputs,'outputname',states);

t = 0 : 0.001 : 10;
r = zeros(2, length(t));
X0 = [0.1; 0; 0.1; 0];
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
[AX,~,~] = plotyy(t,I2 * u_control(:,1),t,I2 * u_control(:,2),'plot');
set(get(AX(1),'Ylabel'),'String','Torque M_x (Nm)')
set(get(AX(2),'Ylabel'),'String','Torque M_y (Nm)')
title('Inputs')




% %% Discretization
% T = 1 / 100; % Sampling Period
% SSd = c2d(SS, T, 'zoh');
% Gz = tf(SSd);
% 
% [Kd, Pd, ed] = lqr(SSd.A, SSd.B, Q, R, N);
% 
% Acd = A - B * K;
% SScld = ss(Ac, Bc, C, D,'statename',states,'inputname',inputs,'outputname',states);
% 
% [yd,t,xd]=lsim(SScl,r',t, X0);
% 
% % Feedback Control Law u = - K x
% ud_control = - (Kd * xd')';
% 
% figure;
% [AX,~,~] = plotyy(t,yd(:,1),t,yd(:,2),'stem');
% set(get(AX(1),'Ylabel'),'String','phi_x (rad)')
% set(get(AX(2),'Ylabel'),'String','phi_y (rad)')
% title('Step response')
% 
% % Torques
% figure;
% [AX,~,~] = plotyy(t,ud_control(:,1),t,ud_control(:,2),'stem');
% set(get(AX(1),'Ylabel'),'String','Torque M_x (Nm)')
% set(get(AX(2),'Ylabel'),'String','Torque M_y (Nm)')
% title('Inputs')
