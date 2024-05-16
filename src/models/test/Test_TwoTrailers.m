model = Mdl_TwoTrailersCLASS();

dt = 0.05;

reference_input = [1;1];

v = (reference_input(1) + reference_input(2))/2;

w = (reference_input(1) - reference_input(2))/(model.distance*2);

x2 = -model.Lt1 - model.L2;

x3 = x2 - model.Lt2 - model.L3;

reference_state = [0;0;0;x2;0;0;x3;0;0];

A = model.SystemMatrix(reference_state, reference_input, dt, model.p);

B = model.ControlMatrix(reference_state, reference_input, dt, model.p);

% disp(A);
% 
% disp(B);
% 
% new_state = A*reference_state  + B*reference_input;
% 
new_state = model.Function(reference_state, reference_input, dt, model.p);
% 
disp(new_state);
