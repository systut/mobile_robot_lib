model = Mdl_TractorTrailerCLASS();

dt = 0.05;

reference_input = [1;1];

reference_state = [0;0;0;0;0;0;0];

[A, B] = model.ConstructSystemMatrix(reference_state, reference_input, dt);

disp(A);

disp(B);

state = model.Update(reference_state, reference_input, dt);

disp(state);