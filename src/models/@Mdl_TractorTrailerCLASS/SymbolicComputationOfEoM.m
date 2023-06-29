function SymbolicComputationOfEoM()
    %SYMBOLICCOMPUTATIONOFEOM Summary of this function goes here
    %   Detailed explanation goes here
    syms length_front length_back slip_right slip_left wheel_distance dt

    p = [length_front, length_back, slip_right, slip_left, wheel_distance];

    nx = 6;

    nu = 2;
    
    % [x, y, theta]
    x = sym('x',[nx,1]);
    
    % [v_r, v_l]
    u = sym('u',[nu,1]);

    % [x, y, theta]
    state = sym('x',[nx,1]);
    
    % [v_r, v_l]
    input = sym('u',[nu,1]);

    v = ((1 - slip_right) * u(1) + (1 - slip_left) * u(2))/2;

    w = ((1 - slip_right) * u(1) - (1 - slip_left) * u(2))/(2 * wheel_distance);
        
    delta = x(6) - x(3);

    dfdt = [cos(x(3)) * v;
            sin(x(3)) * v;
            w; 
            v * cos(x(6)) * cos(delta) + w * length_back * cos(x(6)) * sin(delta);
            v * sin(x(6)) * cos(delta) + w * length_back * sin(x(6)) * sin(delta);
          - v * (1/length_front) * sin(delta) - w * (length_back/length_front) * cos(delta)];
    
    f = state + subs(dfdt,[x;u],[state;input]) * dt;

    A_linearized   = subs(jacobian(dfdt, x),[x; u],[state;input]);

    B_linearized   = subs(jacobian(dfdt, u),[x; u],[state;input]);

    A   = eye(nx) + A_linearized * dt;

    B   = B_linearized * dt;

    %% Create MATLAB-functions:
    % identify the current file location, to place all functions there
    filename = mfilename('fullpath');
    [filepath,~,~] = fileparts(filename);
    % dummy variable for obj, so that these can be used within the CLASS
    syms obj 
    % for dynamics:
    matlabFunction(A,'file',[filepath,'/SystemMatrix'],'vars',{obj, state, input, dt, p});
    matlabFunction(B,'file',[filepath,'/ControlMatrix'],'vars',{obj, state, input, dt, p});
    matlabFunction(f,'file',[filepath,'/Function'],'vars',{obj, state, input, dt, p});

end

