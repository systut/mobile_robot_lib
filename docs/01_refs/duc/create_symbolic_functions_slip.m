% >>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>
%% simulate and linearize ODEs using the symbolic toolbox
% >>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>
clear all
close all
clc

if ~exist('functions', 'dir')
  mkdir('functions')
end
addpath('functions')

% >>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>
%% Init nonlinear ODE & linearization
% >>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>

% define symbolic variables
x = sym('x',[3,1]);  %[x; y; theta]
u = sym('u',[2,1]);  %[v_r,v_l];
syms l h             % wheel to center, sample time
syms i_r i_l
parameters = [l,h,i_r,i_l];

x_lin = sym('x_lin',[3,1]);
u_lin = sym('u_lin',[2,1]);
x_k   = sym('x_k',[3,1]);
u_k   = sym('u_k',[2,1]);

% nonlinear ODE dx/dt = f(x,u)
f = [cos(x(3))*((1-i_r)*u(1) + (1-i_l)*u(2))/2;
     sin(x(3))*((1-i_r)*u(1) + (1-i_l)*u(2))/2;
     ((1-i_r)*u(1) - (1-i_l)*u(2))/(2*l)];
 
% linearized discrete time system
A_lin   = subs(jacobian(f,x),[x;u],[x_lin;u_lin]);
B_lin   = subs(jacobian(f,u),[x;u],[x_lin;u_lin]);

A_d     = eye(length(x)) + h*A_lin;
B_d     = h*B_lin;

% nonlinear discrete time system
f_d     = x_k + h*subs(f,[x;u],[x_k;u_k]);

% export matlab functions
matlabFunction(f,  'File','functions/f_num_slip'  ,'Vars',{x,u,parameters});
matlabFunction(f_d,'File','functions/f_d_num_slip','Vars',{x_k,u_k,parameters});
matlabFunction(A_d,'File','functions/A_d_num_slip','Vars',{x_lin,u_lin,parameters});
matlabFunction(B_d,'File','functions/B_d_num_slip','Vars',{x_lin,u_lin,parameters});

% >>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>
%% Simulate reference trajectory
% >>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>

% load trajectory
%load('reference.mat');
ref   = reference();
ref   = ref';
t_ref = ref(1,:);
x_ref = ref(2:4,:);
u_ref = ref(5:6,:);

% parameters
l = 0.53 / 2;   % wheel to center
h = 0.05;       % time increment
i_r = 0.3;
i_l = 0.3;
p = [l,h,i_r,i_l];      % parameter vector

% try different points of linearization
idx_lin = 1;
% idx_lin = length(t_ref)*1/4;  
% idx_lin = length(t_ref)*1/2;  
% idx_lin = length(t_ref)*3/4;  

% init simulation
x_sim      = 0*x_ref;           % get vector with same length as x_ref
x_sim(:,1) = x_ref(:,1);        % initial state of simulation
% x_lin      = x_ref(:,idx_lin);  % point around which system will be linearized
% u_lin      = u_ref(:,idx_lin);

%linearization (fixed point)
% A = A_d_num(x_lin,u_lin,p);
% B = B_d_num(x_lin,u_lin,p);
%     
% dx = x_sim - x_lin;
% du = u_ref - u_lin;

x_sim_lin = 0*x_ref;
x_sim_lin(:,1) = x_ref(:,1);

% simulate nonlinear & linear discrete time system
for k = 1:length(t_ref)-1
    x_lin = x_ref(:, k);
    u_lin = u_ref(:, k);
    
    A = A_d_num_slip(x_lin,u_lin,p);
    B = B_d_num_slip(x_lin,u_lin,p);
    
    dx = x_sim(:,k) - x_lin;
    du = u_ref(:,k) - u_lin;
    
    x_sim(:,k+1) = f_d_num_slip(x_sim(:,k),u_ref(:,k),p);
    dx    = A*dx + B*du;
    
    x_sim_lin(:,k+1) = dx + x_lin;
end

% add linearization point
% x_sim_lin = dx + repmat(x_lin,[1,length(dx)]);

% plot
fig = figure(31);
fig.Color = 'w';

cla;
hold on;
plot(x_sim(1,:),x_sim(2,:),'linewidth',2)
plot(x_sim_lin(1,:),x_sim_lin(2,:),'linewidth',2)
plot(x_ref(1,:),x_ref(2,:),'k--','linewidth',2)
grid on;
box on;
hold off;
xlim([-0.6,0.6])
ylim([-2.5,0.5])
ax  = gca();
ax.TickLabelInterpreter = 'latex';
xlabel('position $x_1(t)$ in $\mathrm{m}$','interpreter','latex')
ylabel('position $x_2(t)$ in $\mathrm{m}$','interpreter','latex')
legend({'nonlinear ODE','linear ODE','reference'},'interpreter','latex',...
                                                  'orientation','horizontal',...
                                                  'location','south')
                                                  
% >>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>
%% reference trajectory
% >>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>
function ref = reference()
    t = linspace(0,60,20*60); % should take 60s to complete with 20 Hz sampling rate
    l = 0.53 / 2;   % wheel to center

    % Lemniscate of gerono, adapted so that one period takes 60s
    x_d = sin(2*(2*pi)/60*t) / 2;
    y_d = cos((2*pi)/60*t)-1;
    % First derivative of adapted lemniscate of gerono
    x_d_dot = (2*pi)/60*cos(2*(2*pi)/60*t);
    y_d_dot = -(2*pi)/60*sin((2*pi)/60*t);
    % Second derivative of adapted lemniscate of gerono
    x_d_ddot = -2*(2*pi)/60*(2*pi)/60*sin(2*(2*pi)/60*t);
    y_d_ddot = -(2*pi)/60*(2*pi)/60*cos((2*pi)/60*t);

    % Reference for theta and theta_dot, calculated from the previous
    % references and the system model without any slip
    theta_d = atan2(y_d_dot, x_d_dot);
    theta_d_dot = (y_d_ddot .* x_d_dot - x_d_ddot .* y_d_dot) ./ (x_d_dot.^2 + y_d_dot.^2);

    % Reference for v_l_d and v_r_d
    v_l_d = -l*theta_d_dot + sqrt(x_d_dot.^2 + y_d_dot.^2);
    v_r_d = l*theta_d_dot + sqrt(x_d_dot.^2 + y_d_dot.^2);
    % Save as timeseries to use as input for dynamics.slx
    v_l_ts = timeseries(v_l_d, t);
    v_r_ts = timeseries(v_r_d, t);

%     % Plot the references for x, y and theta
%     figure
%     plot(x_d, y_d);
%     figure
%     plot(t, theta_d);
%     hold on
%     plot(t, theta_d_dot);
%     grid on

    ref = [t', x_d', y_d', theta_d', v_r_d', v_l_d'];
end