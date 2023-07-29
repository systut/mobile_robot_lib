classdef Ctrl_MPControlCLASS
    %CTRL_MPCONTROLCLASS Summary of this class goes here
    %   Detailed explanation goes here
    
    properties
        % MPC Horizon
        N = 100;  

        % Initiate optimization parameters
        tol_opt = 1e-8;

        options;

        % Intial input and state for optimization
        initial_x;
        initial_u;
        nx;
        nu;
        
        % Constrain parameter value
        a_max = 4.0;
        b_max = pi/2;
        
        previous_predicted_x_out;
        controller_failed = false;
    end
   
    % Private Properties
    properties (SetAccess = private, GetAccess = private)
        model;        % A model object
        trajectory;
    end
    
    methods (Access = public)
        function obj = Ctrl_MPControlCLASS(model, trajectory)
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

            obj.nx = obj.model.nx;

            obj.nu = obj.model.nu;



            obj.initial_u = ones(obj.nu*obj.N, 1);

            obj.initial_x = ones(obj.nx*(obj.N+1), 1);
        end
        
        function [status, u_out, obj, predicted_x_out] = Loop(obj, y, u, index)
            %Loop Summary of this method goes here
            %   Detailed explanation goes here
            status = true;

            iterations = length(obj.trajectory.x) - obj.N;
            
            if index > iterations | obj.controller_failed == true
                status = false;

                u_out = zeros(2, 1);
                predicted_x_out = obj.previous_predicted_x_out;

                return
            end

            % Get references for the current MPC loop
            x_in_horizon = obj.trajectory.x(:, index:index+obj.N);
            u_in_horizon = obj.trajectory.u(:, index:index+obj.N);

            [Aeq, beq] = obj.ConstructEqualityConstraint(x_in_horizon, u_in_horizon, y);

            [Ain, bin] = obj.ConstructInequalityConstraint(x_in_horizon, u_in_horizon, y);

            [H, f] = obj.ConstructCostFunction();

            % Solve optimization problem
            % Initial value for decision variable
            z0 = [obj.initial_x; obj.initial_u];

            [solution,~,exit_flag,~] = quadprog(H, f, Ain, bin, Aeq, beq, [], [], z0, obj.options);
            
            if exit_flag ~= 1
                exit_flag
                obj.controller_failed = true;
                status = false;
                u_out = zeros(obj.nu,1);
                predicted_x_out = obj.previous_predicted_x_out;
                return
            end 

            % Derive optimal predicted state and input sequence
            x_optimal = solution(1:obj.nx*(obj.N+1), 1);
            u_optimal = solution(obj.nx*(obj.N+1)+1:end, 1);
            u_optimal_reshaped = reshape(u_optimal, obj.nu, obj.N);
            
            % Get control input needed for the simulated WMR
            u_out = u_optimal_reshaped(:,1) + u_in_horizon(:, 1);
            predicted_x_out = x_optimal + reshape(x_in_horizon,[],1);

            obj.previous_predicted_x_out = predicted_x_out;

            obj.initial_x = [y - x_in_horizon(:, obj.N); x_optimal(obj.nx+1:end)];
            obj.initial_u = [u_optimal(obj.nu+1:end); zeros(obj.nu, 1)]; 
        end

        %% Inequality constraint
        function [Ain, bin]= ConstructInequalityConstraint(obj, x_in_horizon, u_in_horizon, y)
            Ain = [];
            bin = [];
            % inequality constrains on acceleration
            H_u = [ 1,  0, -1,  0; 
                    -1,  0,  1,  0;
                    0,  1,  0, -1;
                    0, -1,  0,  1];

            tmp_Ain = zeros(size(H_u, 1)*(obj.N-1), obj.nx*(obj.N+1) + obj.nu*obj.N);
            tmp_bin = zeros(size(H_u, 1)*(obj.N-1), 1);

            k_u = 4.0 * obj.trajectory.dt * ones(4,1);
            
            for k = 0:obj.N-2
                
                tmp_Ain(1 + k*size(H_u, 1) : (k+1)*size(H_u, 1), ...
                    obj.nx*(obj.N+1) + 1 + k*obj.nu : obj.nx*(obj.N+1) + size(H_u, 2) + k*obj.nu ) ...
                         = H_u;
                
                tmp_bin(1+ k*size(H_u, 1) : (k+1)*size(H_u, 1), 1) = k_u;
            end
            clear('k')
            Ain = [Ain; tmp_Ain];
            bin = [bin; tmp_bin]; 

            % Inequality constrains for state

            S_x = [ 0, 0, 1, 0, 0, -1;
                   0, 0, -1, 0, 0, 1];

            tmp_Ain = zeros(size(S_x, 1)*(obj.N+1), obj.nx*(obj.N+1) + obj.nu*obj.N);
            tmp_bin = zeros(size(S_x, 1)*(obj.N+1), 1);

            for k = 1:obj.N
                
                x = x_in_horizon(:, 1+k);
                
                tmp_Ain(1 + k*size(S_x, 1) : (k+1)*size(S_x, 1), ...
                        1 + k*obj.nx : size(S_x, 2) + k*obj.nx ) ...
                         = S_x;
                
                tmp_bin(1+ k*size(S_x, 1) : (k+1)*size(S_x, 1), 1) = ...
                            [obj.b_max - (x(3,1)-x(6,1)); ...
                             obj.b_max + (x(3,1)-x(6,1))  
                            ];
            end
            clear('k')
            Ain = [Ain; tmp_Ain];
            bin = [bin; tmp_bin];           

        end

        function [Aeq, beq] = ConstructEqualityConstraint(obj, x_in_horizon, u_in_horizon, y)
                % Build equality constraints (dynamics)
                Aeq = zeros(obj.nx*obj.N, obj.nx*(obj.N+1) + obj.nu*obj.N); 
                beq = zeros(obj.nx*obj.N, 1);

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
                
                Aeq = [[eye(obj.nx), zeros(obj.nx, obj.nx*obj.N+obj.nu*obj.N)]; Aeq];

                % Equality constraints (initial constraint)

                beq= [y - x_in_horizon(:, 1); beq];
        end

        %% Cost function
        function [H, f] = ConstructCostFunction(obj)
            % weighting matrices for robot state > 0 (1x3)
            Q = diag([.001, .001, .001, 50., 50., 50.]);
    
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
            Qstack = blkdiag(zeros(obj.nx), Qstack);
            H = blkdiag(Qstack, Rstack);
            
            % system function (1x64)
            f = zeros(1, obj.nx * (obj.N + 1) + obj.nu * obj.N);
        end
    end
end

