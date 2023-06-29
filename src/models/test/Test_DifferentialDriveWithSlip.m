model = Mdl_DifferentialDriveWithSlipCLASS();

dt = 0.05;

reference_input = [1;1];

reference_state = [0;0;0];

A = model.SystemMatrix(reference_state, reference_input, dt, model.p);

B = model.ControlMatrix(reference_state, reference_input, dt, model.p);

disp(A);

disp(B);

F = model.Function(reference_state, reference_input, dt, model.p);

disp(F);