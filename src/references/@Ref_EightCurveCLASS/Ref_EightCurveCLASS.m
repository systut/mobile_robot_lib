classdef Ref_EightCurveCLASS
    properties
        % Params
        R = 20; 
        tMAX;
        dt;
        % States
        x;
        dxdt;
        ddxddt;
        % Input
        u;
        u_norm;
        x_out;
        % Timestamp
        t;
        % Class
        mode;
        className;
        folderPath;
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

            obj.mode = "normal";

            obj.className = class(obj);

            [obj.folderPath] = fileparts(which(obj.className));
        end

        function obj = Generate(obj)
            %LE Summary of this function goes here
            %   Detailed explanation goes here
            if strcmp(obj.mode, "load")

                obj = obj.Load();

                return;
            end

            obj.t = linspace(0, obj.tMAX, (1/obj.dt) * obj.tMAX); % should take 60s to complete with 20 Hz sampling rate
            w = (2*pi) / obj.tMAX;
            
            % Lemniscate of gerono, adapted so that one period takes 60s
            obj.x = [obj.R * sin(2*w*obj.t) / 2; obj.R * (cos(w*obj.t)-1); atan2(-obj.R * w * sin(w*obj.t), obj.R * w * cos(2*w*obj.t))];
        
            % First derivative of adapted lemniscate of gerono
            obj.dxdt = [obj.R * w * cos(2*w*obj.t); -obj.R * w * sin(w*obj.t)];
            
            % Second derivative of adapted lemniscate of gerono
            obj.ddxddt = [-2 * obj.R * w * w * sin(2*w*obj.t); -obj.R * w * w * cos(w*obj.t)];

            if isa(obj.model, 'Mdl_BicycleCLASS')
                v = sqrt(obj.dxdt(1, :).^2 + obj.dxdt(2, :).^2);
                
                dthetadt = (obj.ddxddt(2, :) .* obj.dxdt(1, :) - obj.ddxddt(1, :) .* obj.dxdt(2, :)) ./ (obj.dxdt(1, :).^2 + obj.dxdt(2, :).^2);

                delta = atan(obj.model.length_base * (obj.ddxddt(2, :) .* obj.dxdt(1, :) - obj.ddxddt(1, :) .* obj.dxdt(2, :)) ./ (v.^3));
            
                ddeltadt = zeros(1, length(obj.t));

                for index = 2:length(obj.t)
                    ddeltadt(1, index) = (delta(1, index) - delta(1, index-1)) / obj.dt; 
                end

                obj.u = [v; ddeltadt]; 

                obj.u_norm = [v; dthetadt]; 

            elseif isa(obj.model, 'Mdl_TractorTrailerCLASS')
                w2 = (obj.ddxddt(2, :) .* obj.dxdt(1, :) - obj.ddxddt(1, :) .* obj.dxdt(2, :)) ./ (obj.dxdt(1, :).^2 + obj.dxdt(2, :).^2);
                
                v2 = sqrt(obj.dxdt(1, :).^2 + obj.dxdt(2, :).^2);

                w1 = zeros(1, length(obj.t));
                
                v1 = zeros(1, length(obj.t));
    
                obj.x_out = [zeros(3, length(obj.t));obj.x];
                % Initial state
                obj.x_out(1:3,1) = [ obj.model.length_front +  obj.model.length_back; 0 ; 0];
                
                w1(1) = -obj.model.length_front*(1/obj.model.length_back)*w2(1);

                v1(1) = v2(1);

                for index = 2:length(obj.t)

                    obj.x_out(1, index) = obj.x_out(1, index-1) + v1(index-1) * cos(obj.x_out(3, index-1)) * obj.dt;

                    obj.x_out(2, index) = obj.x_out(2, index-1) + v1(index-1) * sin(obj.x_out(3, index-1)) * obj.dt;

                    obj.x_out(3, index) = obj.x_out(3, index-1) + w1(index-1) * obj.dt;

                    gamma = obj.x_out(6, index) - obj.x_out(3, index);

                    w1(index) = (1/obj.model.length_back) * ( v2(index)*sin(gamma) + obj.model.length_front * w2(index) * cos(gamma));
                    
                    v1(index) = v2(index)*cos(gamma) - obj.model.length_front*w2(index)*sin(gamma);

                end

                obj.x = obj.x_out;

                v_r = obj.model.distance * w1 + v1;
                v_l = -obj.model.distance * w1 + v1;

                obj.u = [v_r; v_l];

                obj.u_norm = [v1;w1];

            else
                dthetadt = (obj.ddxddt(2, :) .* obj.dxdt(1, :) - obj.ddxddt(1, :) .* obj.dxdt(2, :)) ./ (obj.dxdt(1, :).^2 + obj.dxdt(2, :).^2);

                v_r = obj.model.distance * dthetadt + sqrt(obj.dxdt(1, :).^2 + obj.dxdt(2, :).^2);
                
                v_l = -obj.model.distance * dthetadt + sqrt(obj.dxdt(1, :).^2 + obj.dxdt(2, :).^2);
                
                v = sqrt(obj.dxdt(1, :).^2 + obj.dxdt(2, :).^2);

                obj.u = [v_r; v_l];

                obj.u_norm = [v; dthetadt];
            end
        end

                function Save(obj)
            %METHOD1 Summary of this method goes here
            %   Detailed explanation goes here
            data = [obj.t', obj.x', obj.u'];

            data = round(data, 6);

            output = [num2cell(data)];
            
            fileName = append(class(obj), '.csv');

            filePath = append(obj.folderPath, '/', fileName);

            writecell(output, filePath); % introduced in Matlab 2019a
        end

        function obj = Load(obj)
            %METHOD1 Summary of this method goes here
            %   Detailed explanation goes here
            fileName = append(class(obj), '.csv');

            filePath = append(obj.folderPath, '/', fileName);

            data = readmatrix(filePath);

            obj.t = data(:, 1)';
            
            obj.x = data(:, 2:2+obj.model.nx-1)';

            obj.u = data(:, 2+obj.model.nx:2+obj.model.nx+obj.model.nu-1)';
        end
    end

    methods (Access = private)
        function [v_out, w_out, theta_out] = solveW(obj, v, w, theta, theta_)
            syms temp;
            
            theta_out_ = theta_ + temp*obj.dt;

            equation = v*sin(theta - theta_out_) + obj.model.length_front*w*cos(theta - theta_out_) - obj.model.length_back*temp == 0;

            solution = vpasolve(equation, temp, -1);

            w_out = solution;

            v_out = subs(v*cos(theta - theta_out_) - obj.model.length_front*w*sin(theta - theta_out_), temp, w_out); 
        
            theta_out = subs(theta_out_, temp, w_out);
        end   
    end
end



 