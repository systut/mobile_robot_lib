clear
clc
close all

%% Motion model
model = Mdl_TwoTrailersCLASS();
% model = Mdl_DifferentialDriveCLASS();
%% Trajectory Generation
dt = 0.05;
T_max = 60;
t = linspace(0,T_max,20*T_max); % should take 60s to complete with 20 Hz sampling rate
l = model.distance ;   % wheel to center
w = 2*pi/T_max;    % angular velocity
dir = 1;        % direction of traversion, only +/- 1 possilbe
R = 1;          % 'radius'

% Lemniscate of gerono with constant angular velocity w
x_d = R*sin(2*w*t) / 2;
y_d = R*(cos(w*t)-1);
% First derivative of adapted lemniscate of gerono
x_d_dot = R*w*cos(2*w*t);
y_d_dot = -R*w*sin(w*t);
% Second derivative of adapted lemniscate of gerono
x_d_ddot = -2*R*w*w*sin(2*w*t);
y_d_ddot = -R*w*w*cos(w*t);

% Reference for theta and theta_dot, calculated from the previous
% references and the system model without slip
theta_d = atan2(y_d_dot, x_d_dot);
theta_d_dot = (y_d_ddot .* x_d_dot - x_d_ddot .* y_d_dot) ./ (x_d_dot.^2 + y_d_dot.^2);

v_d = sqrt(x_d_dot.^2 + y_d_dot.^2);

% Reference for v_r and v_l
v_r_d = l*theta_d_dot + dir * v_d;
v_l_d = -l*theta_d_dot + dir * v_d;

%% Initial condition
initial_state = [0;0;0;- (model.Lt1+model.L2);0;0; -(model.Lt1+model.L2+model.Lt2+model.L3);0;0];

% initial_state = [0;0;0];
% 
%% Simulate
x_k = initial_state;
x_out = x_k;
u_out = [];
for i = 1:length(t)
    u_out(:,i) = [v_r_d(1,i); v_l_d(1,i)];
    x_k1 = model.Function(x_out(:,i), u_out(:,i), dt, model.p);
    % x_k_new = model.SystemMatrix(x_out(:,i), u_out(:,i), dt, model.p)*x_k + model.ControlMatrix(x_out(:,i), u_out(:,i), dt, model.p)*u_k;
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
ax.TickLabelInterpreter = 'latex';
xlabel('position $x$ in $\mathrm{m}$', 'interpreter', 'latex');
ylabel('position $y$ in $\mathrm{m}$', 'interpreter', 'latex');
legend('reference', 'tractor', 'trailer_1', 'trailer_2', 'interpreter', 'latex', 'orientation','vertical',...
                                            'location','southeast');
hold off;
