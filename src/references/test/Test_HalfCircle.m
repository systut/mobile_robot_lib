% Params
R = 60;
dt = 0.05;
tSTART = 0;
tMAX = 60;

model = Mdl_DifferentialDriveCLASS();

trajectory = Ref_HalfCircleCLASS(model);

trajectory.tMAX   = tMAX;                      % maximum simulation time
trajectory.dt = dt; 
trajectory.R = R; 
trajectory = trajectory.Generate();

figure;
grey = [0.2431,    0.2667,    0.2980];
plot(trajectory.x(1,:), trajectory.x(2,:), '--', 'Color', grey, 'linewidth', 1.5), grid on, hold on,