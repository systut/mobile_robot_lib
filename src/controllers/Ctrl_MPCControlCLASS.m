classdef Ctrl_MPCControlCLASS
    %CTRL_MPCONTROLCLASS Summary of this class goes here
    %   Detailed explanation goes here
    
    properties
        % MPC Horizon
        N;  

        % Initiate optimization parameters
        tol_opt = 1e-8;

        options;

        % Matrix for inequality constraint 
        Ai;
        bi

        % Intial input and state for optimization
        initial_x;
        initial_u;
        nx;
        nu;
    end
   
    % Private Properties
    properties (SetAccess = private, GetAccess = private)
        model;        % A model object
        trajectory;
    end
    
    methods (Access = public)
        function obj = Ctrl_MPCControlCLASS(model, trajectory)
            %CTRL_MPCCONTROLCLASS Construct an instance of this class
            %   Detailed explanation goes here
            obj.model = model;

            obj.trajectory = trajectory;

            obj.options = optimset('Display','off',...
            'TolFun', obj.tol_opt,...
            'MaxIter', 10000,...
            'TolConSQP', 1e-6); 
        end

        function obj = Init(obj)
            obj.ConstructInequalityConstraint();

            obj.nx = obj.model.nx + 1;

            obj.nu = obj.model.nu;

            obj.N = 20;

            obj.initial_u = zeros(obj.nu*obj.N, 1);

            obj.initial_x = zeros(obj.nx*obj.N, 1);
        end
        
        function [status, u_out, obj] = Loop(obj, y, u, index)
            %Loop Summary of this method goes here
            %   Detailed explanation goes here
            status = true;

            iterations = length(obj.trajectory.x) - obj.N;
            
            if index > iterations
                status = false;

                u_out = zeros(2, 1);
                
                return
            end

            % Get references for the current MPC loop
            x_in_horizon = obj.trajectory.x(:, index:index+obj.N);
            u_in_horizon = obj.trajectory.u(:, index:index+obj.N);

            [Aeq, beq] = obj.ConstructEqualityConstraint(x_in_horizon, u_in_horizon, y);

            [H, f] = obj.ConstructCostFunction();

            % Solve optimization problem
            % Initial value for decision variable
            z0 = [obj.initial_x; obj.initial_u];
            [solution,~,exit_flag,~] = quadprog(H, f, obj.Ai, obj.bi, Aeq, beq, [], [], z0, obj.options);
            
            if exit_flag ~= 1
                status = false;
                return
            end 

            % Derive optimal predicted state and input sequence
            x_optimal = solution(1:obj.nx*(obj.N+1), 1);
            u_optimal = solution(obj.nx*(obj.N+1)+1:end, 1);
            u_optimal_reshaped = reshape(u_optimal, obj.nu, obj.N);
            
            % Get control input needed for the simulated WMR
            u_out = u_optimal_reshaped(:,1) + u_in_horizon(:, 1);

            obj.initial_x = [x_optimal(obj.nx+1:end); y - x_in_horizon(:, obj.N)];
            obj.initial_u = [u_optimal(obj.nu+1:end); zeros(obj.nu, 1)]; 
        end

        %% Inequality constraint
        function obj = ConstructInequalityConstraint(obj)
            H_u = [ 1,  0, -1,  0;
                   -1,  0,  1,  0;
                    0,  1,  0, -1;
                    0, -1,  0,  1];

            k_u = 4.0 * obj.trajectory.dt * ones(4,1);
            
            Au = [];
            bu = [];

            for k=0:obj.N-2
                Au = [Au, zeros(size(Au,1), obj.nu);
                      zeros(size(H_u,1),k*obj.nu), H_u];
                bu = [bu; k_u];
            end

            clear('k')
            
            obj.Ai = [zeros(size(Au,1),obj.nx*(obj.N+1)), Au];
            obj.bi = bu;
        end

        function [Aeq, beq] = ConstructEqualityConstraint(obj, x_in_horizon, u_in_horizon, y)
                % Build equality constraints (dynamics)
                Aeq = zeros(obj.nx*(obj.N+1), obj.nx*(obj.N+1) + obj.nu*obj.N);
                beq = zeros(obj.nx*(obj.N+1), 1);

                for k = 0:obj.N-1
                    % System matrices for system linearized around the reference
                    x = x_in_horizon(:, 1+k); % Point on state ref to linearize around
                    u = u_in_horizon(:, 1+k); % Point on input ref to linearize around

                    A = obj.model.SystemMatrix(x, u, obj.trajectory.dt, obj.model.p_without_slip);
                    B = obj.model.ControlMatrix(x, u, obj.trajectory.dt, obj.model.p_without_slip);
            
                    % This part gets multiplied with the state part of the decision
                    % variable, thus the Ad matrices
                    Aeq(obj.nx*k+1:obj.nx*(k+1), ...
                        1:obj.nx*(obj.N+1)) = [
                        zeros(obj.nx, obj.nx*k), ...
                        A, ...
                        -eye(obj.nx), ...
                        zeros(obj.nx, obj.nx*(obj.N-1-k))];
                    % This part gets multiplied with the input part of the decision
                    % variable, thus the Bd matrices
                    Aeq(obj.nx*k+1:obj.nx*(k+1), ...
                        obj.nx*(obj.N+1)+1:end) = [
                        zeros(obj.nx, obj.nu*k), ...
                        B, ...
                        zeros(obj.nx, obj.nu*(obj.N-1-k))];
                end
                clear('k')
                
                % Equality constraints (initial constraint)
                Aeq(obj.nx*obj.N+1:obj.nx*(obj.N+1), :) = [eye(obj.nx), zeros(obj.nx, obj.nx*obj.N+obj.nu*obj.N)];
                
                beq(obj.nx*obj.N+1:obj.nx*(obj.N+1))    = y - x_in_horizon(:, 1);
        end

        %% Cost function
        function [H, f] = ConstructCostFunction(obj)
            % weighting matrices for robot state > 0 (1x3)
            Q = diag([.3, .3, .3, 50, 50, .3]);
    
            % weighting matrices for control input  > 0 (1x2)
            R = diag([0.00001, 0.00001]);  

            Qstack = [];
            Rstack = [];
            
            for k = 1:obj.N
                % Stage cost : weighting matrics for input (in discrete horizon) (40x40)
                Qstack = blkdiag(Qstack, Q);
                % Stage cost : weighting matrics for output (in discrete horizon) (20x20)
                Rstack = blkdiag(Rstack, R);
            end
            clear('k')

            % terminal cost = 0 (At end of horizon)
            Qstack = blkdiag(Qstack, zeros(obj.nx));
            H = blkdiag(Qstack, Rstack);
            
            % system function (1x64)
            f = zeros(1, obj.nx * (obj.N + 1) + obj.nu * obj.N);
        end

    end
end

