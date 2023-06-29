classdef Ctrl_MPControlCLASS
    %CTRL_MPCONTROLCLASS Summary of this class goes here
    %   Detailed explanation goes here
    
    properties
        % MPC Horizon
        N;
        
        % weighting matrices for robot state > 0 (1x3)
        Q = diag([50, 50, 3, 3]);

        % weighting matrices for control input  > 0 (1x2)
        R = diag([0.1, 0.1]);    

        % Initiate optimization parameters
        tol_opt       = 1e-8;
        options = optimset('Display','off',...
            'TolFun', tol_opt,...
            'MaxIter', 10000,...
            'TolConSQP', 1e-6); 

        % Limit
        v_max = 0.7069;
        a_max = 44.3182;
        delta_max = pi/2;
        ddeltadt_max = 3.2580;
    end
   
    % Private Properties
    properties (SetAccess = private, GetAccess = private)
        model;        % A model object
        trajectory;
    end
    
    methods
        function obj = Ctrl_MPControlCLASS(model, trajectory)
            %CTRL_MPCCONTROLCLASS Construct an instance of this class
            %   Detailed explanation goes here
            obj.model = model;

            obj.trajectory = trajectory;
        end
        
        function input = Loop(obj, q, dqdt, ddqddt)
            %METHOD1 Summary of this method goes here
            %   Detailed explanation goes here
            reference_input = obj.ConstructReferenceInput(q, dqdt, ddqddt);

        end

        %% Inequality constraint
        function [Ai, bi] = ConstructInequalityConstraint(obj)
            H_u = [ 1,  0, -1,  0;
                   -1,  0,  1,  0;
                    0,  1,  0, -1;
                    0, -1,  0,  1];

            k_u = 4.0*delta*ones(4,1);
            
            Au = [];
            bu = [];

            for k=0:obj.N-2
                Au = [Au, zeros(size(Au,1), obj.model.ntau);
                      zeros(size(H_u,1),k*obj.model.ntau), H_u];
                bu = [bu; k_u];
            end

            clear('k')
            
            Ai = [zeros(size(Au,1),obj.model.nq*(obj.N+1)), Au];
            bi = bu;
        end

        function [Aeq, beq] = ConstructEqualityConstraintByStep(obj, index, reference_input, reference_state)
                % Get references for the current MPC loop
                state_ref_mpc = reference_state(:, index:index+obj.N);
                input_ref_mpc = reference_input(:, index:index+obj.N);
                
                % Build equality constraints (dynamics)
                Aeq = zeros(obj.model.nq*(obj.N+1), obj.model.nq*(obj.N+1) + obj.model.ntau*obj.N);
                beq = zeros(obj.model.nq*(obj.N+1), 1);
                
                state_estim = measurement;
                p_pred = [p_lin, zeros(1,2)];
                
                for k = 0:obj.N-1
                    % System matrices for system linearized around the reference
                    state_lin = state_ref_mpc(:, 1+k); % Point on state ref to linearize around
                    input_lin = input_ref_mpc(:, 1+k); % Point on input ref to linearize around
                    
                    A_d = A_d_num_slip(state_lin, input_lin, p_pred);
                    B_d = B_d_num_slip(state_lin, input_lin, p_pred);
            
                    % This part gets multiplied with the state part of the decision
                    % variable, thus the Ad matrices
                    Aeq(n*k+1:n*(k+1), 1:n*(N+1)) = [zeros(n, n*k), A_d, -eye(n), zeros(n, n*(N-1-k))];
                    % This part gets multiplied with the input part of the decision
                    % variable, thus the Bd matrices
                    Aeq(n*k+1:n*(k+1), n*(N+1)+1:end) = [zeros(n, m*k), B_d, zeros(n, m*(N-1-k))];
                end
                clear('k')
                
                % Overwrite state_lin and input_lin from the previous loop to set it to
                % values for the initial timestep of the prediction
                state_lin = state_ref_mpc(:, 1);
                input_lin = input_ref_mpc(:, 1);
                
                % Equality constraints (initial constraint)
                Aeq(n*N+1:n*(N+1), :) = [eye(n), zeros(n, n*N+m*N)];
                beq(n*N+1:n*(N+1))    = state_estim - state_lin;
        end

        %% Cost function
        function [H,f] = ConstructCostFunction(obj)
            Qstack = [];
            Rstack = [];

            for k = 1:obj.N
                % Stage cost : weighting matrics for input (in discrete horizon) (40x40)
                Qstack = blkdiag(Qstack, obj.Q);
                % Stage cost : weighting matrics for output (in discrete horizon) (20x20)
                Rstack = blkdiag(Rstack, obj.R);
            end
        
            % terminal cost = 0 (At end of horizon)
            Qstack = blkdiag(Qstack, zeros(obj.model.nq));
        
            % J(x, u_predict) = z'Hz 
            % J : Cost function
            % z : combining of predicted states and input
            % Stage cost : weighting matrics for output and input (in discrete horizon) (64x64)
            H = blkdiag(Qstack, Rstack);
            
            % system function (1x64)
            f = zeros(1, obj.model.nq * (obj.N + 1) + obj.model.ntau * obj.N);
        end
    end

    methods(Static)
        function reference_input = ConstructReferenceInput(q, dqdt, ddqddt)
            dthetadt = (ddqddt(2) .* dqdt(1) - ddqddt(1) .* dqdt(2)) ./ (dqdt(1).^2 + dqdt(2).^2);
            
            % Calculate linear and angular velocity
            v = sqrt(dqdt(1).^2 + dqdt(2).^2);
            w = dthetadt;

            reference_input = [v, w];
        end
    end
end

