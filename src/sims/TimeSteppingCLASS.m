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
        y_out    = [];
        dxdt_out = [];we
        u_out  = [];
        debug = [];
        predicted_x_out = [];
    end
    
    % Private Properties
    properties (SetAccess = private, GetAccess = private)
        model;        % A model object
        controller;
        trajectory;
        observer;
    end

    methods
        function obj = TimeSteppingCLASS(model, trajectory, controller, observer)
            % Constructor creates a simulation for a specific model
            obj.model      = model;
            obj.controller = controller;
            obj.trajectory = trajectory;
            obj.observer   = observer;
        end
        
        function obj = Run(obj, q0)
            obj.t_out    = linspace(obj.tSTART, obj.tMAX, (1/obj.dt) * obj.tMAX);
            nt           = size(obj.t_out,2);
            obj.x_out    = zeros(obj.model.nx,nt);
            obj.y_out    = zeros(obj.model.nx,nt);

            if isa(obj.controller, "Ctrl_MPControlCLASS")
                obj.predicted_x_out = zeros((obj.controller.N+1)*obj.model.nx, nt);
            end

            % Initialize time stepping:
            obj.x_out(:,1)    = q0;
            obj.y_out(:,1)    = q0;

            obj.u_out = zeros(2,nt);
            obj.debug = zeros(2, nt);
            obj.controller = obj.controller.Init();
            
            for i = 1:size(obj.t_out,2)
                yM = obj.y_out(:, i);
                uM = obj.u_out(:, i);
                
                % Controller
                if isa(obj.controller, "Ctrl_MPControlCLASS")
                    [status, obj.u_out(:,i), obj.controller, obj.predicted_x_out(:, i)] = obj.controller.Loop(yM, uM, i);
                else                 
                    [status, obj.u_out(:,i), obj.controller] = obj.controller.Loop(yM, uM, i);
                end

                if ~status
                    obj.u_out(:,i) = zeros(2, 1);
                end
                v = (obj.u_out(1, i) + obj.u_out(2, i))/2;

                w = (obj.u_out(1, i) - obj.u_out(2, i))/(2 * obj.model.distance);

%                 delta = obj.x_out(6, i) - obj.x_out(3, i);

                obj.debug(1, i) = v;

                obj.debug(2, i) = w;
% 
%                 obj.debug(3, i) = v * cos(delta) - w * obj.model.length_back * sin(delta);
%           
%                 obj.debug(4, i) = - v * (1/obj.model.length_front) * sin(delta) - w * (obj.model.length_back/obj.model.length_front) * cos(delta);
%                 
%                 obj.debug(5, i) = delta;

                % Update model
                xM = obj.model.Function(obj.x_out(:, i), obj.u_out(:,i), obj.dt, obj.model.p);
                
                % Add observer
                yM = obj.observer.Observe(xM);
                
                if i < length(obj.t_out)
                    obj.x_out(:, i+1) = xM;
                    obj.y_out(:, i+1) = yM;
                end
            end
        end
    end
end

