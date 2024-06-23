clear
clc
close all

%% Motion model
model = Mdl_TwoTrailersCLASS();

%% Trajectory Generation
dt = 0.05;
T_max = 60;
t = linspace(0,T_max,20*T_max); % should take 60s to complete with 20 Hz sampling rate
l = model.distance ;   % wheel to center
w = 2*pi/T_max;    % angular velocity
dir = 1;        % direction of traversion, only +/- 1 possilbe
R = 4;          % 'radius'

% Lemniscate of gerono with constant angular velocity w
x_d = R*sin(2*w*t) / 2;
y_d = R*(cos(w*t)-1);
% First derivative of adapted lemniscate of gerono
x_d_dot = R*w*cos(2*w*t);
y_d_dot = -R*w*sin(w*t);
% Second derivative of adapted lemniscate of gerono
x_d_ddot = -2*R*w*w*sin(2*w*t);
y_d_ddot = -R*w*w*cos(w*t);

% Reference for velocity v_d
v_d = sqrt(x_d_dot.^2 + y_d_dot.^2);

% Reference for theta and theta_dot, calculated from the previous
% references and the system model without slip
theta_d = atan2(y_d_dot, x_d_dot);
theta_d_dot = (y_d_ddot .* x_d_dot - x_d_ddot .* y_d_dot) ./ (x_d_dot.^2 + y_d_dot.^2);

% w2 = (y_d_ddot .* x_d_dot - x_d_ddot .* y_d_dot) ./ (x_d_dot.^2 + y_d_dot.^2);
% 
% v2 = sqrt(x_d_dot.^2 + y_d_dot.^2);
% 
% w1 = zeros(1, length(t));
% 
% v1 = zeros(1, length(t));
% 
% x = [x_d; y_d; theta_d];
% 
% x_out = [zeros(3, length(t));x];
% 
% % Initial state
% x_out(1:3,1) = [ model.Lt1 +  model.L2; 0 ; 0];
% 
% w1(1) = -model.Lt1*(1/model.L2)*w2(1);
% 
% v1(1) = v2(1);
% 
% for index = 2:length(t)
% 
%     x_out(1, index) = x_out(1, index-1) + v1(index-1) * cos(x_out(3, index-1)) * dt;
% 
%     x_out(2, index) = x_out(2, index-1) + v1(index-1) * sin(x_out(3, index-1)) * dt;
% 
%     x_out(3, index) = x_out(3, index-1) + w1(index-1) * dt;
% 
%     gamma = x_out(6, index) - x_out(3, index);
% 
%     w1(index) = (1/model.L2) * ( v2(index)*sin(gamma) + model.Lt1 * w2(index) * cos(gamma));
% 
%     v1(index) = v2(index)*cos(gamma) - model.Lt1*w2(index)*sin(gamma);
% 
% end
% 
% x = x_out;
% 
% v_r_d = model.distance * w1 + v1;
% v_l_d = -model.distance * w1 + v1;

% Reference for v_r and v_l
v_r_d = l*theta_d_dot + dir * v_d;
v_l_d = -l*theta_d_dot + dir * v_d;

%% Initial condition
initial_state = [0;0;0;- (model.Lt1+model.L2);0;0; -(model.Lt1+model.L2+model.Lt2+model.L3);0;0];

% initial_state = [0;0;0];

%% Simulate
x_k = initial_state;
x_out = x_k;
u_out = [];
for i = 1:length(t)
    u_out(:,i) = [v_r_d(1,i); v_l_d(1,i)];
    x_k1 = model.Function(x_out(:,i), u_out(:,i), dt, model.p);
    x_out(:,i+1) = x_k1;
end
y_out = x_out';

% Position in x-y-plane
grey = [0.2431,    0.2667,    0.2980];
red = [0.7529, 0.0000, 0.000];
green = [0.0000, 0.6902, 0.3137];
yellow = [1.0000, 0.8353, 0.0000];

f1 = figure(1);
f1.Color = 'w';
plot(x_d, y_d, '--', 'Color', grey, 'linewidth', 1.5), grid on, hold on,
plot(y_out(:,1), y_out(:,2), 'Color', green, 'linewidth', 1.5)
plot(y_out(:,4), y_out(:,5), 'Color', red, 'linewidth', 1.5)
plot(y_out(:,7), y_out(:,8), 'Color', yellow, 'linewidth', 1.5)

ax = gca();
ax.FontSize = 15;
ax.TickLabelInterpreter = 'latex';
xlabel('position $x$ in $\mathrm{m}$', 'interpreter', 'latex','FontSize',20);
ylabel('position $y$ in $\mathrm{m}$', 'interpreter', 'latex','FontSize',20);
legend({'reference', 'tractor', 'trailer 1', 'trailer 2'}, ...
        'orientation','vertical',...
        'location','southeast', 'FontSize',12);
hold off;
