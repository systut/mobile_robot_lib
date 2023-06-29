classdef Ref_EightCurveCLASS
    properties
        % Params
        R; 
        tMAX;
        dt;
        % States
        x;
        dxdt;
        ddxddt;
        % Input
        u;
    end

    % Private Properties
    properties (SetAccess = private, GetAccess = private)
        model;        % A model object
    end
    
    
    methods
        function obj = Ref_EightCurveCLASS(model)
            %CTRL_BASECLASS Construct an instance of this class
            %   Detailed explanation goes here
            obj.model = model;
        end

        function obj = Generate(obj)
            %LE Summary of this function goes here
            %   Detailed explanation goes here
            t = linspace(0, obj.tMAX, (1/obj.dt) * obj.tMAX); % should take 60s to complete with 20 Hz sampling rate
            w = (2*pi) / obj.tMAX;
            
            % Lemniscate of gerono, adapted so that one period takes 60s
            obj.x = [obj.R * sin(2*w*t) / 2; obj.R * (cos(w*t)-1)];
        
            % First derivative of adapted lemniscate of gerono
            obj.dxdt = [obj.R * w * cos(2*w*t); -obj.R * w * sin(w*t)];
        
            % Second derivative of adapted lemniscate of gerono
            obj.ddxddt = [-2 * obj.R * w * w * sin(2*w*t); -obj.R * w * w * cos(w*t)];

            if isa(obj.model,'Mdl_BicycleCLASS')
                v = sqrt(obj.dxdt(1, :).^2 + obj.dxdt(2, :).^2);

                delta = atan(obj.model.length_base * (obj.ddxddt(2, :) .* obj.dxdt(1, :) - obj.ddxddt(1, :) .* obj.dxdt(2, :)) ./ (v.^3));

                ddeltadt = zeros(1, length(t));

                for i = 1:length(t)-1
                    ddeltadt(1, i) = (delta(1, i+1) - delta(1, i)) / obj.dt; 
                end

                obj.u = [v, ddeltadt]; 

            else
                dthetadt = (obj.ddxddt(2, :) .* obj.dxdt(1, :) - obj.ddxddt(1, :) .* obj.dxdt(2, :)) ./ (obj.dxdt(1, :).^2 + obj.dxdt(2, :).^2);

                v_r = obj.model.distance * dthetadt + sqrt(obj.dxdt(1, :).^2 + obj.dxdt(2, :).^2);
                v_l = -obj.model.distance * dthetadt + sqrt(obj.dxdt(1, :).^2 + obj.dxdt(2, :).^2);
    
                obj.u = [v_r, v_l];
            end
        end
    end
end



 