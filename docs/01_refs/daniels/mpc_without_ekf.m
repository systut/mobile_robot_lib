% This script implements linear mpc for a differential drive robot.
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

% Set options
tol_opt       = 1e-8;
options = optimset('Display','off',...
    'TolFun', tol_opt,...
    'MaxIter', 10000,...
    'TolConSQP', 1e-6);


% Robot parameters
l = 0.53 / 2;   % wheel to center


% Reference trajectory
ref = csvread('reference.csv',2,0);
ref = ref';
t_ref = ref(1,:);
reference_x = ref(2,:);
reference_y = ref(3,:);
reference_theta = ref(4,:);
state_ref = [reference_x; reference_y; reference_theta];

reference_v_r = ref(5,:);
reference_v_l = ref(6,:);
input_ref = [reference_v_r; reference_v_l];
%input_ref = 0.1*ones(2,length(reference_v_r));

slip_ref = [zeros(2, floor(length(t_ref)/3)), 0.3*ones(2, ceil(2*length(t_ref)/3))];


% MPC parameters
delta = 0.05;   % Sampling time
N = 10;          % discrete horizon
T = N*delta;    % continuous horizon

MPCIterations = length(t_ref) - N;  % Number of iterations of solver

p_lin = [l, delta];

% System dimensions
n = 3;
m = 2;
o = 3;

% Initial system conditions
tmeasure = 0.0;
state_real = [0; 0; 0];     % initial state of the system
measurement = state_real;

input0 = zeros(m*N, 1);
state0 = zeros(n*N, 1);

% ==============================================
% Implement the MPC iteration
% ==============================================

% Use the quadprog command to solve the optimization problem
% quadprog has the following command
%
% z = quadprog(H,f,A,b,Aeq,beq,[],[],z0,options)
%
% z:        optimal solution
%
% H:        quadratic part of the cost
% f:        linear part of the cost
% Ai, bi:   inequality constraints A z <= b
% Aeq, beq: equality constraints Aeq z = beq
% z0:       inital guess for the optimization
% options:  options for the solver

% Set variables for output
t = [];
state = [];
input = [];
measurements = [];
estimates = [];

% Cost function
% Cost parameters
Q_mpc = diag([50, 50, .3]);
R_mpc = diag([0.1, 0.1]);

% Stage cost
Qstack = [];
Rstack = [];
for k = 1:N
    Qstack = blkdiag(Qstack, Q_mpc);
    Rstack = blkdiag(Rstack, R_mpc);
end
clear('k')

% terminal cost = 0
Qstack = blkdiag(Qstack, zeros(n));

H = blkdiag(Qstack, Rstack);
f = zeros(1, n*(N+1)+m*N);


% Inequality constraints go here, as they stay constant
% Used for state and input constraints, no constraints for now
% H_u = [ 1,  0;
%        -1,  0;
%         0,  1;
%         0, -1];
% k_u = 0.5*ones(4,1);

% Inequality constraints
% Au = [];
% bu = [];
% for k = 1:N
%     Au = blkdiag(Au, H_u);
%     bu = [bu; k_u];
% end
% clear('k')
% 
% Ai = [zeros(size(Au,1),n*(N+1)), Au];
% bi = bu;

% Ai = [];
% bi = [];


H_u = [ 1,  0, -1,  0;
       -1,  0,  1,  0;
        0,  1,  0, -1;
        0, -1,  0,  1];
k_u = 4.0*delta*ones(4,1);

Au = [];
bu = [];
for k=0:N-2
    Au = [Au, zeros(size(Au,1), m);
          zeros(size(H_u,1),k*m), H_u];
    bu = [bu; k_u];
end
clear('k')

Ai = [zeros(size(Au,1),n*(N+1)), Au];
bi = bu;


% Preallocation of variables
slip_estim = zeros(2,length(ref(1,:))-N);


% Set up random number generator for measurement noise
rng('default');     % for reproducibility
noise_mu = zeros(3,1);
noise_sigma =  diag([1, 1, 2])*1e-4;
noise_cov = chol(noise_sigma);


for ii = 1:MPCIterations
    % Measure the time
    t_Start = tic;
    
    % Get references for the current MPC loop
    state_ref_mpc = state_ref(:, ii:ii+N);
    input_ref_mpc = input_ref(:, ii:ii+N);
    
    % Build equality constraints (dynamics)
    Aeq = zeros(n*(N+1), n*(N+1) + m*N);
    beq = zeros(n*(N+1), 1);
    
    state_estim = measurement;
    p_pred = [p_lin, zeros(1,2)];
    
    for k = 0:N-1
        % System matrices for system linearized around the reference
        state_lin = state_ref_mpc(:, 1+k); % Point on state ref to linearize around
        input_lin = input_ref_mpc(:, 1+k); % Point on input ref to linearize around
        
        A_d = A_d_num_slip(state_lin, input_lin, p_pred);
        B_d = B_d_num_slip(state_lin, input_lin, p_pred);

        % This part gets multiplied with the state part of the decision
        % variable, thus the Ad matrices
        Aeq(n*k+1:n*(k+1), 1:n*(N+1)) = [zeros(n, n*k), A_d, -eye(n), zeros(n, n*(N-1-k))];
        % This part gets multiplied with the input part of the decision
        % variable, thus the Bd matrices
        Aeq(n*k+1:n*(k+1), n*(N+1)+1:end) = [zeros(n, m*k), B_d, zeros(n, m*(N-1-k))];
    end
    clear('k')
    
    % Overwrite state_lin and input_lin from the previous loop to set it to
    % values for the initial timestep of the prediction
    state_lin = state_ref_mpc(:, 1);
    input_lin = input_ref_mpc(:, 1);
    
    % Equality constraints (initial constraint)
    Aeq(n*N+1:n*(N+1), :) = [eye(n), zeros(n, n*N+m*N)];
    beq(n*N+1:n*(N+1))    = state_estim - state_lin;
    
    % Terminal constraints (Zero-terminal-constraint)
    % Careful: Problem might not be feasible for some initial values
    % Without feasibility no stability
    % Aeq(n*(N+1)+1:n*(N+2), :) = [zeros(n, n*N), eye(n), zeros(n, m*N)];
    % beq(n*(N+1)+1:n*(N+2))    = [0; 0; 0];

    % Solve optimization problem
    % Initial value for decision variable
    z0 = [state0; input0];
    [solutionOL,~,exit_flag,~] = quadprog(H, f, Ai, bi, Aeq, beq, [], [], z0, options);
    
    if exit_flag ~= 1
        exit_flag
    end
    
    % Derive optimal predicted state and input sequence
    state_OL_tilde = solutionOL(1:n*(N+1), 1);
    input_OL_tilde = solutionOL(n*(N+1)+1:end, 1);
    state_OL = reshape(state_OL_tilde, n, N+1);
    input_OL = reshape(input_OL_tilde, m, N);
    
    % Get control input needed for the simulated WMR
    input_wmr = input_OL(:,1) + input_lin;
    
    t_Elapsed = toc( t_Start );
    
    % Store closed-loop data;
    t = [ t, tmeasure ];
    state = [ state, state_real ];
    input = [ input, input_wmr ];
    
    % Update the closed-loop system
    p_real = [p_lin, slip_ref(:,ii)'];
    state_real = f_d_num_slip(state_real, input_wmr, p_real);
    %measure_noise = randn(1,3)*noise_cov;
    measure_noise = zeros(1,3);
    measure_noise = noise_mu + measure_noise';
    measurement = state_real + measure_noise;
    tmeasure = tmeasure + delta;
    
    measurements = [ measurements, measurement ];
    
    % Prepare warmstart solution for next time step (take the endpiece of the optimal open-loop solution 
    % and add a last piece)
    state0 = [state_OL_tilde(n+1:end); state_estim - state_ref_mpc(:, N)];
    input0 = [input_OL_tilde(m+1:end); zeros(m, 1)];    
end


% >>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>
%% Plot results
% >>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>
% Corporate design colors
grey = [0.2431,    0.2667,    0.2980];
red = [0.7529, 0.0000, 0.000];
green = [0.0000, 0.6902, 0.3137];
blue = [0.0000, 0.3176, 0.6196];
yellow = [1.0000, 0.8353, 0.0000];


% Position in x-y-plane
f1 = figure(1);
f1.Color = 'w';
plot(reference_x, reference_y, '--', 'Color', grey, 'linewidth', 1.5), grid on, hold on,
plot(measurements(1,:), measurements(2,:), '--', 'Color', green, 'linewidth', 1.5)
plot(state(1,:), state(2,:), '-', 'Color', blue, 'linewidth', 1.5)
hold off;
box on;
ylim([-3.0,0.5]);

ax = gca();
ax.TickLabelInterpreter = 'latex';
xlabel('position $x$ in $\mathrm{m}$', 'interpreter', 'latex');
ylabel('position $y$ in $\mathrm{m}$', 'interpreter', 'latex');
legend('reference', 'measurement', 'real state', 'interpreter', 'latex', 'orientation','vertical',...
                                            'location','southeast');

cleanfigure('targetResolution', 300)
matlab2tikz('figures/position_wout_ekf.tex','width','\fwidth', 'encoding', 'utf8')


% Error in the position
% Calculate state error in every point
error = zeros(length(state(1,:)),1);
for ii = 1:length(state(1,:))
    error(ii) = norm(state_ref(1:2,ii) - state(1:2,ii));
end
f2 = figure(2);
f2.Color = 'w';
plot(t_ref(1:end-N), error, '-', 'Color', blue, 'linewidth', 1.5), grid on,

ax = gca();
ax.TickLabelInterpreter = 'latex';
xlabel('time $t$ in $\mathrm{s}$', 'interpreter', 'latex');
ylabel('error in position in $\mathrm{m}$', 'interpreter', 'latex');

cleanfigure('targetResolution', 300)
matlab2tikz('figures/error_pos_wout_ekf.tex','width','\fwidth', 'encoding', 'utf8')


% Error in the orientation
f3 = figure(3);
f3.Color = 'w';
plot(t_ref(1:end-N),state_ref(3,1:end-N)-state(3,:), '-', 'Color', blue, 'linewidth', 1.5), grid on, hold on,

ax = gca();
ax.TickLabelInterpreter = 'latex';
xlabel('time $t$ in $\mathrm{s}$', 'interpreter', 'latex');
ylabel('error in orientation in $\mathrm{rad}$', 'interpreter', 'latex');

cleanfigure('targetResolution', 300)
matlab2tikz('figures/error_theta_wout_ekf.tex','width','\fwidth', 'encoding', 'utf8')


% Input for the right wheel
f4 = figure(4);
f4.Color = 'w';
plot(t_ref(1:end-N),input(1,:), '-', 'Color', blue, 'linewidth', 1.5), grid on, hold on,

ax = gca();
ax.TickLabelInterpreter = 'latex';
xlabel('time $t$ in $\mathrm{s}$', 'interpreter', 'latex');
ylabel('wheel velocity $v_r$ in $\mathrm{ms^{-1}}$', 'interpreter', 'latex');

cleanfigure('targetResolution', 300)
matlab2tikz('figures/input_right_wout_ekf.tex','width','\fwidth', 'encoding', 'utf8')

% Input for the left wheel
f5 = figure(5);
f5.Color = 'w';
plot(t_ref(1:end-N),input(2,:), '-', 'Color', blue, 'linewidth', 1.5), grid on, hold on,

ax = gca();
ax.TickLabelInterpreter = 'latex';
xlabel('time $t$ in $\mathrm{s}$', 'interpreter', 'latex');
ylabel('wheel velocity $v_l$ in $\mathrm{ms^{-1}}$', 'interpreter', 'latex');

cleanfigure('targetResolution', 300)
matlab2tikz('figures/input_left_wout_ekf.tex','width','\fwidth', 'encoding', 'utf8')


% csvwrite(['state_input_for_slip_', num2str(s_l), '.csv'], [state', input'])
