classdef Ctrl_StanleyCLASS
    %CTRL_STANLEYCLASS Summary of this class goes here
    %   Detailed explanation goes here
    
    properties
        % Speed proportional gain
        Kp = 1;

        Ke = 0.5
    end

    % Private Properties
    properties (SetAccess = private, GetAccess = private)
        model;        % A model object
        trajectory;
    end
    
    methods
        function obj = Ctrl_StanleyCLASS(model, trajectory)
            %CTRL_STANLEYCLASS Construct an instance of this class
            %   Detailed explanation goes here
            obj.model = model;

            obj.trajectory = trajectory;
        end
        
        function obj = Init(obj)
            %METHOD1 Summary of this method goes here
            %   Detailed explanation goes here
            
%             obj.target_index = obj.pCalculateTargetIndex(obj.trajectory.x, obj.trajectory.x(:, 1));
        end

        function [status, u_out, obj] = Loop(obj, y, u, index)
            %Loop Summary of this method goes here
            %   Detailed explanation goes here
            status = true;

            a = obj.pCalculateProportionalAcceleration(obj.trajectory.u(:, index), u);
            
            heading_error = obj.normalize(obj.trajectory.x(3) - y(3));

            crosstrack_error = obj.pCalculateCrosstrackError(y, u, index);

            v = (u(1) + u(2)) / 2 + a * obj.trajectory.dt;
            
            w = heading_error + crosstrack_error;

            v_r = obj.model.distance * w + v;

            v_l = -obj.model.distance * w + v;

            u_out = [v_r; v_l];
        end
    end

    methods (Access = private)
        function crosstrack_error = pCalculateCrosstrackError(obj, current_x, current_u, index)
            v = (current_u(1) + current_u(2)) / 2;
            
            error = obj.pCalculateDistance(obj.trajectory.x(:, index), current_x);

            crosstrack_error = atan2(obj.Ke * error, v);
        end

        function a = pCalculateProportionalAcceleration(obj, reference_u, current_u)
            reference_v = (reference_u(1) + reference_u(2)) / 2;

            v = (current_u(1) + current_u(2)) / 2;

            a = obj.Kp * (reference_v - v);
        end

        function target_index = pCalculateTargetIndex(obj, current_x)
            distance = obj.pCalculateDistance(obj.trajectory.x, current_x);

            [value, target_index] = min(distance);
        end

        function distance = pCalculateDistance(obj, reference_x, current_x)
            distance = current_x - reference_x;

            distance = hypot(distance(1,:), distance(2,:));
        end
    end

    methods (Static)
        function angle = normalize(angle)
            while angle > pi
                angle = angle - 2.0 * pi;

            end
        
            while angle < -pi
                angle = angle + 2.0 * pi;

            end
        end
    end
end

