function SymbolicComputationOfEoM()
    %SYMBOLICCOMPUTATIONOFEOM Summary of this function goes here
    %   Detailed explanation goes here
    syms length_base dt

    p = [length_base];

    nx = 4;

    nu = 2;
    
    % [x, y, theta]
    x = sym('x',[nx,1]);
    
    % [v_r, v_l]
    u = sym('u',[nu,1]);

    % [x, y, theta]
    state = sym('x',[nx,1]);
    
    % [v_r, v_l]
    input = sym('u',[nu,1]);

    dfdt = [u(1)*cos(x(3));                  % v * cos(theta)
            u(1)*sin(x(3));                  % v * sin(theta)
            u(1)*tan(x(4))/length_base;      % v * tan(delta) / L
            u(2)];                           % delta dot

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

