%% Inverted Pendulum Derivations
%% SMC
clc;clear;close('all');

syms x1 x2 x3 x4 u Ke r_pulley bc bp mc mp g Lp Jp real
x1_dot = x2;
x2_dot = (Ke*u/r_pulley - bc*x2 - mp*Lp*x4^2*sin(x3) + mp*Lp*cos(x3)*(-bp*x4 + mp*g*Lp*sin(x3))/(mp*Lp^2 + Jp))*1/(mc + mp - mp^2*Lp^2*cos(x3)^2/(mp*Lp^2 + Jp));
x3_dot = x4;
x4_dot = (-bp*x4 + mp*g*Lp*sin(x3) + mp*Lp*cos(x3)*(Ke*u/r_pulley - bc*x2 - mp*Lp*x4^2*sin(x3))/(mc + mp))*1/(mp*Lp^2 + Jp - mp^2*Lp^2*cos(x3)^2/(mc + mp));

x_dot = [x1_dot; x2_dot; x3_dot; x4_dot];
x_vars = [x1;x2;x3;x4];

F = subs(x_dot,u,0);
G = jacobian(x_dot,u);