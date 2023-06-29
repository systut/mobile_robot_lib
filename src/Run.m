%% Initial Setup
% Make a clean sweep:
clear all
close all
clc

% Initial state
q0 = [0;0;0;0;0;0];
q0 = [0;0;0;0];

% Params
R = 1;
dt = 0.05;
tSTART = 0;
tMAX = 60;

%% Set up model:
model = Mdl_TractorTrailerCLASS();
model = Mdl_BicycleCLASS();

%% Set up trajectory:
trajectory = Ref_EightCurveCLASS(model);
trajectory.tMAX   = tMAX;                      % maximum simulation time
trajectory.dt = dt; 
trajectory.R = R; 
trajectory = trajectory.Generate();

%% Set up controller:
controller = Ctrl_FeedForwardCLASS(model);

%% Set up simultion: 
simulation = TimeSteppingCLASS(model,trajectory, controller);
simulation.tSTART = tSTART;                       % initial time
simulation.tMAX   = tMAX;                      % maximum simulation time
simulation.dt = dt; 

%% Run simulation
simulation = simulation.Run(q0);

%% Set up animation:
animation = AnimationCLASS(model, trajectory, controller, simulation);
animation.Animate();

