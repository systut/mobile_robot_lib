classdef Ctrl_PurepursuitCLASS
    %CTRL_PUREPURSUITCLASS Summary of this class goes here
    %   Detailed explanation goes here
    
    properties
        % Lookahead distance
        initial_lookahead = 2.0;
        
        % Speed proportional gain
        Kp = 1.0;
        
        % Look forward gain
        kL = 1.0;
        
        % Nearest target x 
        nearest_x;
        nearest_index;
    end
    
    % Private Properties
    properties (SetAccess = private, GetAccess = private)
        model;        % A model object
        trajectory;
    end

    methods (Access = public)
        function obj = Ctrl_PurepursuitCLASS(model, trajectory)
            %CTRL_PUREPURSUITCLASS Construct an instance of this class
            %   Detailed explanation goes here
            obj.model = model;
            obj.trajectory = trajectory;
        end
        
        function obj = Init(obj)
            %METHOD1 Summary of this method goes here
            %   Detailed explanation goes here
            
        end

        function [status, u_out, obj] = Loop(obj, y, u, index)
            %METHOD1 Summary of this method goes here
            %   Detailed explanation goes here
            status = true;

            [index, lookahead] = obj.pSearchTargetIndex(y, u);

            alpha = atan2(obj.trajectory.x(1, index) - y(1), obj.trajectory.x(2, index) - y(2)) - y(3);

            a = obj.pCalculateProportionalAcceleration(obj.trajectory.u(:, index), u);

            v = (u(1) + u(2)) / 2 + a * obj.trajectory.dt;

            w = v * 2.0 * sin(alpha) / lookahead;

            v_r = obj.model.distance * w + v;

            v_l = -obj.model.distance * w + v;

            u_out = [v_r; v_l];
        end
    end

    methods (Access = private)
        function [index, lookahead, obj] = pSearchTargetIndex(obj, current_x, current_u)
            if isempty(obj.nearest_x)                
                all_distance = obj.pCalculateDistance(obj.trajectory.x, current_x);

                [value, index] = min(all_distance);

                obj.nearest_index = index;

            else
                index = obj.nearest_index;

                this_distance = obj.pCalculateDistance(obj.trajectory.x(:, index), current_x);

                while True
                    next_distance = obj.pCalculateDistance(obj.trajectory.x(:, index+1), current_x);

                    if this_distance < next_distance
                        break
                    end
                    
                    if (index + 1) < len(obj.trajectory.x)
                        index = index + 1;
                    end

                    this_distance = next_distance;
                end

                obj.nearest_index = index;
               
            end

            v = (current_u(1) + current_u(2)) / 2;

            lookahead = obj.kL * v + obj.initial_lookahead;
            
            distance = obj.pCalculateDistance(obj.trajectory.x(:, index), current_x);

            while lookahead > distance
                if (index + 1) >= length(obj.trajectory.x)
                    break 
                end

                index = index + 1;

            end
        end

        function distance = pCalculateDistance(obj, reference_x, current_x)
            distance = current_x - reference_x;

            distance = hypot(distance(1,:), distance(2,:));
        end
        

        function a = pCalculateProportionalAcceleration(obj, reference_u, current_u)
            reference_v = (reference_u(1) + reference_u(2)) / 2;

            v = (current_u(1) + current_u(2)) / 2;

            a = obj.Kp * (reference_v - v);
        end
    end

    
end

