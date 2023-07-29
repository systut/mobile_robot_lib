classdef Ref_CoveragePath2CLASS
    properties
        % Params
        R = 1; 
        tMAX;
        dt;
        % States
        x;
        dxdt;
        ddxddt;
        % Input
        u;
        u_norm;
        u_norm_back;
        x_out;
        % Timestamp
        t;
        % Class
        mode;
        className;
        folderPath;
        x_start = 0.;
        y_start = 0.;
    end

    % Private Properties
    properties (SetAccess = private, GetAccess = private)
        model;        % A model object
    end
    
    methods
        function obj = Ref_CoveragePath2CLASS(model)
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
            v = 2*pi*obj.R/obj.tMAX; % m/s

            obj.x = [];
            obj.dxdt = [];
            obj.ddxddt = [];

            for index=1:length(obj.t)/4
                [x_, dxdt_, ddxddt_] = obj.GenerateStraightLine(v, [obj.x_start;obj.y_start; 0], obj.t(index));

                obj.x = [obj.x, x_];

                obj.dxdt = [obj.dxdt, dxdt_];

                obj.ddxddt = [obj.ddxddt, ddxddt_];
            end
                    
            for index=1:length(obj.t)/2
                [x_, dxdt_, ddxddt_] = obj.GenerateClockwiseHalfCircle(v, [obj.x_start + pi*obj.R/2 ; obj.y_start; 0], obj.R, obj.t(index));

                obj.x = [obj.x, x_];

                obj.dxdt = [obj.dxdt, dxdt_];

                obj.ddxddt = [obj.ddxddt, ddxddt_];
            end


            for index=1:length(obj.t)/4
                [x_, dxdt_, ddxddt_] = obj.GenerateStraightLine(-v,[obj.x_start + pi*obj.R/2; obj.y_start + 2*obj.R; pi],obj.t(index));

                obj.x = [obj.x, x_];

                obj.dxdt = [obj.dxdt, dxdt_];

                obj.ddxddt = [obj.ddxddt, ddxddt_];
            end
    
            if isa(obj.model, 'aMdl_BicycleCLASS')
                v = sqrt(obj.dxdt(1, :).^2 + obj.dxdt(2, :).^2);

                delta = atan(obj.model.length_base * (obj.ddxddt(2, :) .* obj.dxdt(1, :) - obj.ddxddt(1, :) .* obj.dxdt(2, :)) ./ (v.^3));

                ddeltadt = zeros(1, length(obj.t));

                for index = 1:length(obj.t)-1
                    ddeltadt(1, index) = (delta(1, index+1) - delta(1, index)) / obj.dt; 
                end

                obj.u = [v; ddeltadt]; 

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

                    w1(index) = (-1/obj.model.length_back) * ( v2(index)*sin(gamma) + obj.model.length_front * w2(index) * cos(gamma));
                    
                    v1(index) = v2(index)*cos(gamma) - obj.model.length_front*w2(index)*sin(gamma);
                end

                obj.x = obj.x_out;

                v_r = obj.model.distance * w1 + v1;
                v_l = -obj.model.distance * w1 + v1;

                obj.u = [v_r; v_l];

                obj.u_norm = [v1;w1];

                obj.u_norm_back = [v1;w1;v2;w2;obj.x_out(6, :)-obj.x_out(3, :)];
            else
                dthetadt = (obj.ddxddt(2, :) .* obj.dxdt(1, :) - obj.ddxddt(1, :) .* obj.dxdt(2, :)) ./ (obj.dxdt(1, :).^2 + obj.dxdt(2, :).^2);

                v_r = obj.model.distance * dthetadt + sqrt(obj.dxdt(1, :).^2 + obj.dxdt(2, :).^2);
                v_l = -obj.model.distance * dthetadt + sqrt(obj.dxdt(1, :).^2 + obj.dxdt(2, :).^2);
                   
                obj.u = [v_r; v_l];

                obj.x_out(:, 1) = [0;0;0;-(obj.model.length_back + obj.model.length_front);0;0];

                for index=1:length(obj.t)-1

                     xM = obj.model.Function(obj.x_out(:, index), obj.u(:,index), obj.dt, obj.model.p_without_slip);
                       
                     obj.x_out(:, index+1) = xM;
                end

                obj.x = obj.x_out;

                obj.u_norm = [dthetadt; sqrt(obj.dxdt(1, :).^2 + obj.dxdt(2, :).^2)];
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

    methods(Static)
        function [x, dxdt, ddxddt] = GenerateStraightLine(v, x0, t)
            x      = x0 + [v * t; 0; 0];

            dxdt   = [v; 0];

            ddxddt = [0; 0];
        end

        function [x, dxdt, ddxddt] = GenerateClockwiseHalfCircle(v, x0, R, t)
            x      = x0 + [R*sin((1/R)*v*t); R - R*cos((1/R)*v*t); 1/R*v*t];

            dxdt   = [v * cos((1/R)*v*t) ; v * sin((1/R)*v*t)];

            ddxddt = [-(1/R)*v*v*sin((1/R)*v*t) ; (1/R)*v*v*cos((1/R)*v*t)];
        end  
    end
end



 