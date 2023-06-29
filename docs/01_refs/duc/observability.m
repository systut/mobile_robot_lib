% >>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>
%% extended kalman filter for estimating slip parameters
% >>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>
clear all
close all
clc

if ~exist('functions', 'dir')
  mkdir('functions')
end
addpath('functions')

% >>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>
%% System with extended state vector x_tilde = [x, y, theta, i_r, i_l]
% >>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>

% define symbolic variables
n = 5;  % states
m = 2;  % inputs
x_tilde = sym('x',[n,1]); % [x; y; theta; i_r; i_l]
u = sym('u',[m,1]);     %[v_r,v_l];
syms l h                % wheel to center, sample time
parameters = [l,h];

% nonlinear ODE dx/dt = f(x,u)
f = [cos(x_tilde(3))*((1-x_tilde(4))*u(1) + (1-x_tilde(5))*u(2))/2;
     sin(x_tilde(3))*((1-x_tilde(4))*u(1) + (1-x_tilde(5))*u(2))/2;
     ((1-x_tilde(4))*u(1) - (1-x_tilde(5))*u(2))/(2*l);
     0;
     0];
 
% output relation y = h(x,u)
h1 = x_tilde(1);
h2 = x_tilde(2);
h3 = x_tilde(3);

% >>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>
%% Observability matrix
% >>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>

lie1 = h1;
lie2 = h2;
lie3 = h3;
Q = [jacobian(lie1,x_tilde(4:5)); jacobian(lie2,x_tilde(4:5)); jacobian(lie3,x_tilde(4:5))];
for ii=1:n-1
    lie1 = jacobian(lie1,x_tilde)*f;
    lie2 = jacobian(lie2,x_tilde)*f;
    lie3 = jacobian(lie3,x_tilde)*f;
    Q = [Q; jacobian(lie1,x_tilde(4:5)); jacobian(lie2,x_tilde(4:5)); jacobian(lie3,x_tilde(4:5))];
end

rank(Q)

rank(subs(Q,x_tilde,zeros(5,1)))

rank(subs(Q,u,[1;0]))

