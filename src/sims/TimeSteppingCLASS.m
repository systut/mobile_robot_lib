classdef TimeSteppingCLASS
    %TIMESTEPPINGCLASS Summary of this class goes here
    %   Detailed explanation goes here

    % Public Properties
    properties (SetAccess = public, GetAccess = public)
        % Parameters
        tSTART = 0;
        tMAX   = 60;
        dt     = 0.05;
        % Output:
        t_out    = [];
        x_out    = [];
        dxdt_out = [];
        u_out  = [];
    end
    
    % Private Properties
    properties (SetAccess = private, GetAccess = private)
        model;        % A model object
        controller;
        trajectory;
    end

    methods
        function obj = TimeSteppingCLASS(model, trajectory, controller)
            % Constructor creates a simulation for a specific model
            obj.model      = model;
            obj.controller = controller;
            obj.trajectory = trajectory;
        end
        
        function obj = Run(obj, q0)
            obj.t_out    = obj.tSTART:obj.dt:obj.tMAX;
            nt = size(obj.t_out,2);
            obj.x_out    = zeros(obj.model.nx,nt);
            
            % Initialize time stepping:
            obj.x_out(:,1)    = q0;

            obj.u_out = zeros(2,nt);

            for i = 2:size(obj.t_out,2)
                xM = obj.trajectory.x(:, i-1);
                uM = obj.trajectory.u(:, i-1);

                % Controller
                obj.u_out(:,i) = obj.controller.Loop(xM, uM);

                % Update model
                qM = obj.model.Function(obj.x_out(:, i-1), obj.u_out(:,i), obj.dt, obj.model.p);
                
                obj.x_out(:, i) = qM;
            end
        end
    end
end

