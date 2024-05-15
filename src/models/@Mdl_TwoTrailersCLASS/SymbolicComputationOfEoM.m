function SymbolicComputationOfEoM()
%SYMBOLICCOMPUTATIONOFEOM Summary of this function goes here
%   Detailed explanation goes here
    syms d Lt1 L2 Lt2 L3 s_r s_l dt

    p = [d, Lt1, L2, Lt2, L3, s_r, s_l];

    nx = 9;

    nu = 2;

    % State = [x_i, y_i, theta_i]
    x = sym("x", [nx, 1]);

    % Input = [v_r, v_l]
    u = sym("u", [nu, 1]);

    % State = [x_i, y_i, theta_i]
    state = sym("x", [nx, 1]);

    % Input = [v_r, v_l]
    input = sym('u', [nu,1]);

    %% Motion model

    v = (u(1) + u(2))/2;

    w = (u(1) - u(2))/d;

    theta_12 = x(3) - x(6);

    theta_23 = x(6) - x(9);

    v_2 = v*cos(theta_12) + Lt1*w*sin(theta_12);

    w_2 = (1/L2) * (v * sin(theta_12) - Lt1 * w * cos(theta_12));

    v_3 = v_2 * cos(theta_23) + Lt2 * w_2 * sin(theta_23);

    w_3 = (1/L3) * (v_2 * sin(theta_23) - Lt2 * w * cos(theta_23));

    dfdt = [v * cos(x(3));
            v * sin(x(3));
            w;
            v_2 * cos(x(6));
            v_2 * sin(x(6));
            w_2;
            v_3 * cos(x(9));
            v_3 * sin(x(9));
            w_3];
    
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

