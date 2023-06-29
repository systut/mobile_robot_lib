%% Initial Setup
% Make a clean sweep:
clear all
close all
clc


% Params
R = 20;
dt = 0.05;
tSTART = 0;
tMAX = 60;

%% Set up model:
model = Mdl_DifferentialDriveCLASS();
model = Mdl_TractorTrailerCLASS();
%% Initial state
q0 = zeros(model.nx, 1);

%% Set up trajectory:
trajectory = Ref_EightCurveCLASS(model);
trajectory = Ref_CoveragePathCLASS(model);
trajectory.tMAX   = tMAX;                      % maximum simulation time
trajectory.dt = dt; 
trajectory.R = R; 
trajectory = trajectory.Generate();

%% Set up controller:
controller = Ctrl_FeedForwardCLASS(model);
% controller = Ctrl_MPControlCLASS(model, trajectory);

%% Set up observer; 
observer = Obs_NormalCLASS(model);
observer.noise_sigma =  diag([1, 1, 2, 1, 1, 2])*1e-3;

%% Set up simultion: 
simulation = TimeSteppingCLASS(model, trajectory, controller, observer);
simulation.tSTART = tSTART;                       % initial time
simulation.tMAX   = tMAX;                      % maximum simulation time
simulation.dt = dt; 

%% Run simulation
simulation = simulation.Run(q0);

%% Set up animation:
animation = AnimationCLASS(model, trajectory, controller, simulation);
animation.Animate();

