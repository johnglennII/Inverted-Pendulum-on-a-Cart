%% Inverted Pendulum Sliding Mode Control
clc;clear;close('all');

g = 9.81;                   %gravitational const [m/s^2]
% mc = 700e-3;             %cart mass [kg]
mc = 666e-3;

% no mass
% mp = 101.6762e-3;             %pendulum mass CF [kg]
% Jp = 0.0098563;              %pendlum MOI CF [kg*m^2]
% Lp = 214.4523e-3;           %pendulum Lcom NO MASS [m]

% med mass
mp = 108.8562e-3;
Jp = 0.0139639;
Lp = 266.0696e-3;

% heaviest mass
% mp = 122.277e-3;
% Jp = 0.0203488;
% Lp = 346.2908e-3;

bc = 1.5;                  %cart damping [N*s/m]
bp = 0.01;                 %pendulum damping [N-m*s/rad]
r_pulley = 1.528*25.4/2000; %pulley pitch radius [m]
Ke = .04;                  %motor torque const [N*m/A]
Fc = 3.1;

params.g = g; params.mc = mc; params.mp = mp; params.Jp = Jp; params.bc = bc; 
params.bp = bp; params.Lp = Lp; params.r_pulley = r_pulley; params.Ke = Ke; params.Fc = Fc;

x0 = [0; 0; -9*pi/180; 0];
x_star = [0;0;0;0];
u_star = 0;

n = length(x_star);
dt = 0.0001;
tf = 15;

syms x1 x2 x3 x4 u
x1_dot = x2;
x2_dot = (Ke*u/r_pulley - bc*x2 - mp*Lp*x4^2*sin(x3) + mp*Lp*cos(x3)*(-bp*x4 + mp*g*Lp*sin(x3))/(mp*Lp^2 + Jp))*1/(mc + mp - mp^2*Lp^2*cos(x3)^2/(mp*Lp^2 + Jp));
x3_dot = x4;
x4_dot = (-bp*x4 + mp*g*Lp*sin(x3) + mp*Lp*cos(x3)*(Ke*u/r_pulley - bc*x2 - mp*Lp*x4^2*sin(x3))/(mc + mp))*1/(mp*Lp^2 + Jp - mp^2*Lp^2*cos(x3)^2/(mc + mp));

x_dot = [x1_dot; x2_dot; x3_dot; x4_dot];
x_vars = [x1;x2;x3;x4];

A = double(subs(jacobian(x_dot, x_vars), [x_vars;u], [x_star; u_star]));
B = double(subs(jacobian(x_dot, u), [x_vars;u], [x_star; u_star]));
C = [1,0,0,0; 0,0,1,0];

%check ctrb, obsv
rank(ctrb(A,B));
rank(obsv(A,C));

% --Control Design--
Q = diag([5000, 1, 100, 5]);
R = 1;
[K,~,poles] = lqr(A,B,Q,R);
% poles = [-20, -5, -30, -6];
% K = place(A, B, poles);
disp('=== No Integral Action ===')
disp('K gains:'); disp(K);
disp('CL poles:'); disp(poles);

% Phi = 0.1;
% eta = 105;
Phi = 0.1;
eta = 55;

% --Observer Design--
poles_obsv = 15*max(real(poles))*ones(n,1) - 1.5*[0;1;2;3];
L = place(A', C', poles_obsv)';
disp('L gains:'); disp(round(L'));
disp('Observer poles:'); disp(poles_obsv');

%% Plots
close('all');

try
    t = 0:dt:tf;
    num_samples = length(t);
    y_sim = squeeze(out.y_sim);
    x_history = [y_sim(1,:); nan(1, length(t)); y_sim(2,:); nan(1,length(t))];
    f1 = pend_plots(t, x_history, params, num_samples);
catch
    t = y_actual{1}.Values.Time;
    num_samples = length(t);
    x_history = [y_actual{1}.Values.Data'; nan(1, length(t)); y_actual{2}.Values.Data'; nan(1,length(t))];
    f1 = pend_plots(t, x_history, params, num_samples);
end