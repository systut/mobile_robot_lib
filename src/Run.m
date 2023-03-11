%% Initial Setup
% Make a clean sweep:
clear all
close all
clc

%% Parameter
tSTART = 0;                       % initial time
tMAX   = 15;                      % maximum simulation time
dt = 0.001;                       % size of the time step
t_out    = tSTART:dt:tMAX;
disp(t_out)

% Initial state
q0 = [0;0;0];

% Increment
delta = [0;0;0;0.1;0.1;0];

%% Set up model:
model = Mdl_DifferentialDriveCLASS();

%% Set up controller:
controller = Ctrl_FeedForwardCLASS(model);

%% Run simulation
simulation.Run(q0, delta);

%% Show animation
% animation.Animate();