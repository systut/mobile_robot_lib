% Params
R = 20;
dt = 0.05;
tSTART = 0;
tMAX = 60;

model = Mdl_TractorTrailerCLASS();

trajectory = Ref_CoveragePath2CLASS(model);

trajectory.tMAX   = tMAX;                      % maximum simulation time
trajectory.dt = dt; 
trajectory.R = R; 
trajectory.mode = "normal";
trajectory = trajectory.Generate();

controller = Ctrl_MPControlCLASS(model, trajectory);
controller.N = 10;

controller.Init();