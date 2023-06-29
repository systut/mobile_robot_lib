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

if ~exist('figures', 'dir')
  mkdir('figures')
end

% >>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>
%% System with extended state vector x_tilde = [x, y, theta, i_r, i_l]
% >>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>

% define symbolic variables
x_tilde = sym('x',[5,1]); % [x; y; theta; i_r; i_l]
u = sym('u',[2,1]);     %[v_r,v_l];
syms l h                % wheel to center, sample time
parameters = [l,h];

x_lin = sym('x_lin',[5,1]);
u_lin = sym('u_lin',[2,1]);
x_k   = sym('x_k',[5,1]);
u_k   = sym('u_k',[2,1]);

% nonlinear ODE dx/dt = f(x,u)
f = [cos(x_tilde(3))*((1-x_tilde(4))*u(1) + (1-x_tilde(5))*u(2))/2;
     sin(x_tilde(3))*((1-x_tilde(4))*u(1) + (1-x_tilde(5))*u(2))/2;
     ((1-x_tilde(4))*u(1) - (1-x_tilde(5))*u(2))/(2*l);
     0;
     0];
 
% linearized discrete time system
A_lin   = subs(jacobian(f,x_tilde),[x_tilde;u],[x_lin;u_lin]);
B_lin   = subs(jacobian(f,u),[x_tilde;u],[x_lin;u_lin]);

A_d     = eye(length(x_tilde)) + h*A_lin;
B_d     = h*B_lin;

% nonlinear discrete time system
f_d     = x_k + h*subs(f,[x_tilde;u],[x_k;u_k]);

% export matlab functions
matlabFunction(f,  'File','functions/f_num_ext'  ,'Vars',{x_tilde,u,parameters});
matlabFunction(f_d,'File','functions/f_d_num_ext','Vars',{x_k,u_k,parameters});
matlabFunction(A_d,'File','functions/A_d_num_ext','Vars',{x_lin,u_lin,parameters});
matlabFunction(B_d,'File','functions/B_d_num_ext','Vars',{x_lin,u_lin,parameters});

% >>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>
%% Extended Kalman filter for estimating slip parameters
% >>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>

% load trajectory
ref   = csvread('reference.csv',2,0);
ref   = ref';
t_ref = ref(1,:);
x_ref = ref(2:4,:);
u_ref = ref(5:6,:);
slip_ref = [zeros(2, floor(length(t_ref)/3)), 0.3*ones(2, ceil(2*length(t_ref)/3))];

% Robot parameters
wheel_to_center = 0.53 / 2;

% step width
delta = 0.05;

% Parameter vector
p = [wheel_to_center, delta];

% model matrices
% dynamics      x_sim_k+1 = A_d*x_sim_k + B_d*u_k + W_d*w_k
% measurement:  y_k+1 = C_d*x_sim_k + V_d*v_k
n = 5;      % number of states
m = 2;      % number of inputs
o = 3;      % number of outputs
C_d = [eye(o), zeros(o,n-o)];    % only [x; y; theta] are measured
W_d = eye(n);
V_d = eye(o);

% filter parameters
% covariance of measurement noise
R = diag([1 1 5]);
% covariance of system noise
Q = diag([0.1 0.1 0.2 10 10]);
% initial state
x_tilde_k = [ 0; -0.1; 0; 0.02; 0.02 ]; % state estimated by EKF
x_tilde_simulation = [ 0; 0; 0 ];       % real [x; y; theta] values used for simulation (slip values are taken from slip_ref)
P_k = diag([0.1 0.1 0.1 10 10]);


% Noise
rng('default')      % For reproducibility
noise_mu = zeros(o,1);
noise_sigma =  diag([1, 1, 2])*1e-4;
noise_cov = chol(noise_sigma);
% if 'statistics and machine learning' toolbox is not available, comment
% out the lines from 94 to 96 and line 123. This removes measurement noise,
% but the script can be run


slip_estim = zeros(2,length(t_ref)-1);
state = zeros(3,length(t_ref)-1);
state_estim = zeros(3,length(t_ref)-1);
% simulate nonlinear system for initial value and reference input and
% estimate slip with EKF
for ii=1:length(t_ref)-1
    u_k = u_ref(:,ii);
    
    % prediction / time update eqs
    x_tilde_pred = f_d_num_ext(x_tilde_k, u_k, p);
    
    A_d = A_d_num_ext(x_tilde_k, u_k, p);
    P_pred = A_d*P_k*A_d' + W_d*Q*W_d';
    
    % system simulation
%     x_tilde_measure = x_tilde_measure + 0.0001*randi([-1,1],3,1); % process noise
    x_tilde_real = [x_tilde_simulation(1:3); slip_ref(:,ii)]; % take slip values from the reference
    x_tilde_simulation = f_d_num_ext(x_tilde_real, u_k, p);
    x_tilde_measure = C_d*x_tilde_simulation; % only measure [x; y; theta]
    % Add noise (Both measurement and process noise, since x_tilde_measure
    % is used for simulation...)
    measure_noise = randn(1,3)*noise_cov;
    measure_noise = noise_mu + measure_noise';
    x_tilde_measure = x_tilde_measure + measure_noise;
    state(:, ii) = x_tilde_simulation(1:3);
    
    % correction / measurement update eqs
    % Kalman gain
    K = P_pred*C_d' / ( C_d*P_pred*C_d' + V_d*R*V_d' ); % use / to avoid inv()
    x_tilde_k = x_tilde_pred + K*(x_tilde_measure - C_d*x_tilde_pred);
    P_k = (eye(n) - K*C_d)*P_pred;
    
    state_estim(:,ii) = x_tilde_k(1:3);
    slip_estim(:,ii) = x_tilde_k(4:5);
end

% >>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>
% Plot the results
% >>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>
% Corporate design colors
grey = [0.2431,    0.2667,    0.2980];
red = [0.7529, 0.0000, 0.000];
green = [0.0000, 0.6902, 0.3137];
blue = [0.0000, 0.3176, 0.6196];
yellow = [1.0000, 0.8353, 0.0000];


% Estimated slip value right wheel
f1 = figure(1);
f1.Color = 'w';
plot(t_ref, slip_ref(1,:), '--', 'Color', grey, 'linewidth', 1.5), grid on, hold on,
plot(t_ref(1:end-1), slip_estim(1,:), '-', 'Color', blue, 'linewidth', 1.5)

ax = gca();
ax.TickLabelInterpreter = 'latex';
xlabel('time $t$ in $\mathrm{s}$', 'interpreter', 'latex');
ylabel('slip parameter $s_r$', 'interpreter', 'latex');
legend('reference', 'estimation', 'interpreter', 'latex', 'orientation','vertical',...
                                            'location','southeast');
                                        
matlab2tikz('figures/ekf_slip_right.tex','width','\fwidth', 'encoding', 'utf8')


% Estimated slip value left wheel
f2 = figure(2);
f2.Color = 'w';
plot(t_ref, slip_ref(2,:), '--', 'Color', grey, 'linewidth', 1.5), grid on, hold on,
plot(t_ref(1:end-1), slip_estim(2,:), '-', 'Color', blue, 'linewidth', 1.5)

ax = gca();
ax.TickLabelInterpreter = 'latex';
xlabel('time $t$ in $\mathrm{s}$', 'interpreter', 'latex');
ylabel('slip parameter $s_l$', 'interpreter', 'latex');
legend('reference', 'estimation', 'interpreter', 'latex', 'orientation','vertical',...
                                            'location','southeast');

matlab2tikz('figures/ekf_slip_left.tex','width','\fwidth', 'encoding', 'utf8')


% Estimated and actual system state
f3 = figure(3);
f3.Color = 'w';
plot(state(1,:), state(2,:), '--', 'Color', grey, 'linewidth', 1.5), grid on, hold on,
plot(state_estim(1,:), state_estim(2,:), '-', 'Color', blue, 'linewidth', 1.5)

ax = gca();
ax.TickLabelInterpreter = 'latex';
xlabel('position $x$ in $\mathrm{m}$', 'interpreter', 'latex');
ylabel('position $y$ in $\mathrm{m}$', 'interpreter', 'latex');
legend('nonlinear ODE', 'estimation', 'interpreter', 'latex', 'orientation','vertical',...
                                            'location','southeast');
                                        
matlab2tikz('figures/ekf_state.tex','width','\fwidth', 'encoding', 'utf8')
