% >>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>
%% generate reference trajectory for wmr
% Needs MATLAB 2019a starting in line 133 (because of writecell).
% But you can just use the already generated reverence.csv for the other
% files. Another way is to use csvwrite, but then you cannot include
% headers for the columns (and you have to change all the files using
% reference.csv, because I removed the header line there)
% >>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>
clear all
close all
clc

if ~exist('figures', 'dir')
  mkdir('figures')
end


T_max = 60;
t = linspace(0,T_max,20*T_max); % should take 60s to complete with 20 Hz sampling rate
l = 0.53 / 2;   % wheel to center
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

% Reference for v_r and v_l
v_r_d = l*theta_d_dot + dir * sqrt(x_d_dot.^2 + y_d_dot.^2);
v_l_d = -l*theta_d_dot + dir * sqrt(x_d_dot.^2 + y_d_dot.^2);
% Save as timeseries to use as input for dynamics.slx
v_r_ts = timeseries(v_r_d, t);
v_l_ts = timeseries(v_l_d, t);


% >>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>
%% plot the reference
% >>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>
% Corporate design colors
grey = [0.2431,    0.2667,    0.2980];
red = [0.7529, 0.0000, 0.000];
green = [0.0000, 0.6902, 0.3137];
blue = [0.0000, 0.3176, 0.6196];
yellow = [1.0000, 0.8353, 0.0000];


% xy-plot
f1 = figure(1);
f1.Color = 'w';
plot(x_d, y_d, '-', 'Color', blue, 'linewidth', 1.5);
grid on;
box on;
xlim([-0.6*R,0.6*R])
ylim([-2.5*R,0.5*R])

ax = gca();
ax.TickLabelInterpreter = 'latex';
xlabel('position $x$ in $\mathrm{m}$', 'interpreter', 'latex');
ylabel('position $y$ in $\mathrm{m}$', 'interpreter', 'latex');

% save with matlab2tikz
cleanfigure('targetResolution', 300)
matlab2tikz('figures/xy_reference.tex','width','\fwidth', 'encoding', 'utf8')


% theta and theta_dot plot
f2 = figure(2);
f2.Color = 'w';
hold on
plot(t, theta_d, '-', 'Color', blue, 'linewidth', 1.5);
plot(t, theta_d_dot, '-', 'Color', red, 'linewidth', 1.5);
grid on;
box on;
hold off;
xlim([0, T_max])
ylim([-2.5,2.5])

ax = gca();
ax.TickLabelInterpreter = 'latex';
xlabel('time $t$ in $\mathrm{s}$', 'interpreter', 'latex');
ylabel('$\theta$ in $\mathrm{rad}$, $\dot{\theta}$ in $\mathrm{rad/s}$',...
        'interpreter', 'latex');
legend({'$\theta$', '$\dot{\theta}$'}, 'interpreter', 'latex',...
                                                            'orientation', 'vertical',...
                                                            'location', 'southeast');
% save with matlab2tikz
cleanfigure('targetResolution', 300)
matlab2tikz('figures/theta_reference.tex','width','\fwidth', 'encoding', 'utf8')


% input (v_r, v_l) plot
f3 = figure(3);
f3.Color = 'w';
hold on
plot(t, v_r_d, '-', 'Color', blue, 'linewidth', 1.5);
plot(t, v_l_d, '-', 'Color', red, 'linewidth', 1.5);
grid on;
box on;
hold off;
xlim([0, T_max]);
ylim([-0.2, 0.2]);

ax = gca();
ax.TickLabelInterpreter = 'latex';
xlabel('time $t$ in $\mathrm{s}$', 'interpreter', 'latex');
ylabel('wheel velocities in $\mathrm{m/s}$', 'interpreter', 'latex');
legend({'$v_{r,d}$', '$v_{l,d}$'}, 'interpreter', 'latex',...
                                               'orientation', 'vertical',...
                                               'location', 'southeast');
% save with matlab2tikz
cleanfigure('targetResolution', 300)
matlab2tikz('figures/input_reference.tex','width','\fwidth', 'encoding', 'utf8')


% >>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>
%% save reference
% >>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>
% save as csv for further use and easy import in C++
% ref = [t', x_d', y_d', theta_d', v_r_d', v_l_d'];
% round reference to reduce file size
% ref = round(ref, 6);
% header line to explain columns
% header = {'Time', 'x_d', 'y_d', 'theta_d', 'v_r_d', 'v_l_d'};
% output = [header; num2cell(ref)];
% writecell(output, 'reference.csv'); % introduced in Matlab 2019a

