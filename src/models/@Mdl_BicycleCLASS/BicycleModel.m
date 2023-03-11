classdef BicycleModel
        properties
            % State of kinematic bicycle
            xc;     % x coordinate at center of gravity
            yc;     % y coordinate at center of gravity
            theta;  % (θ) heading angle of bicycle
            delta;  % (δ) steering angle for front wheel
            beta;   % (β) side slip andgle (at center gravity) 
            
            % State: [x,y,θ,δ]'
            % Inputs: [v,φ]' - for: δ_dot = φ

            % Model parameters
            L = 2;                  % wheelbase length
            lr = 1.2;               % length from its center of mass from the rear axle.
            w_max = 1.22;           % maximum turning rate
            sample_time = 0.01
        end
        
        methods
            function obj = BicycleModel()
                obj.xc = 0;
                obj.yc = 0;
                obj.theta = 0;
                obj.delta = 0;
                obj.beta = 0;
            end

            function resetState = reset(obj)
                obj.xc = 0;
                obj.yc = 0;
                obj.theta = 0;
                obj.delta = 0;
                obj.beta = 0;
                resetState = [obj.xc,obj.yc,obj.theta,obj.delta,obj.beta];
            end

            function state = step(obj,v,w)
                if w>obj.w_max
                    w = obj.w_max;
                end

                x_k = obj.xc + v*cos(obj.theta+obj.beta)*obj.sample_time;
                y_k = obj.yc + v*sin(obj.theta+obj.beta)*obj.sample_time;
                theta_k = obj.theta + ((v*cos(obj.beta)*tan(obj.delta))/obj.L)*obj.sample_time;
                delta_k = obj.delta + w*obj.sample_time;
                beta_k = atan(obj.lr*tan(obj.delta)/obj.L);

                obj.xc = x_k;
                obj.yc = y_k;
                obj.theta = theta_k;
                obj.delta = delta_k;
                obj.beta = beta_k;
                state = [obj.xc,obj.yc,obj.theta,obj.delta,obj.beta];

            end
        end
end