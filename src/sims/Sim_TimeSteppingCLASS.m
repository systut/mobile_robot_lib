classdef Sim_TimSteppingCLASS < handle
    %TIMESTEPPINGCLASS Summary of this class goes here
    %   Detailed explanation goes here

    % Public Properties
    properties (SetAccess = public, GetAccess = public)
        % Parameters
        tSTART = 0;
        tMAX   = 1000;
        dt     = 0.001;
        % Output:
        t_out    = [];
        q_out    = [];
        dqdt_out = [];
    end
    
    % Private Properties
    properties (SetAccess = private, GetAccess = private)
        model;        % A model object
        controller;
    end

    methods
        function obj = Sim_TimSteppingCLASS(model,controller)
            % Constructor creates a simulation for a specific model
            obj.model      = model;
            obj.controller = controller;
        end
        
        function Run(obj, q0)
            obj.t_out    = obj.tSTART:obj.dt:obj.tMAX;
            nt = size(obj.t_out,2);
            obj.q_out    = zeros(obj.model.nq,nt);
            obj.dqdt_out = zeros(obj.model.nq,nt);
    
            % Initialize time stepping:
            obj.q_out(:,1)    = q0;
            obj.dqdt_out(:,1) = dqdt0;

            obj.u = zeros(obj.model.nq,nt);
            for i = 2:size(obj.t_out,2)
                % Controller
                obj.tau_out(:,i) = obj.controller.Loop(t_out[i]);
                
            end
        end
    end
end

