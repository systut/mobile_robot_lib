classdef AnimationCLASS
    %TIMESTEPPINGCLASS Summary of this class goes here
    %   Detailed explanation goes here

    % Public Properties
    properties (SetAccess = public, GetAccess = public)
        % Constants  
        grey = [0.2431,    0.2667,    0.2980];
        green = [0.0000, 0.6902, 0.3137];
        red = [0.6902, 0, 0.3137];
        blue = [0.0000, 0.3176, 0.6196];
        % Rate (10FPS)
        rate = 0.1;
        slowDown = 1;
    end
    
    % Private Properties
    properties (SetAccess = private, GetAccess = private)
        model;        % A model object
        controller;
        trajectory;
        simulation;
        fig; % The output window
        ax;  % The output axis
        debug;
        robots;
    end

    methods
        function obj = AnimationCLASS(model, trajectory, controller, simulation)
            % Constructor creates a simulation for a specific model
            obj.model      = model;

            obj.controller = controller;

            obj.trajectory = trajectory;

            obj.simulation = simulation;

            % Set up output figure:
            obj.fig = figure;
            obj.ax = axes;
            hold on
            box on
            grid on
            axis equal
            set(obj.fig,'Name','2D Animation');
            set(obj.fig,'Color','w');
            obj.debug = true;
        end

        function obj = Animate(obj)
            % Bring window to front:
            figure(obj.fig);
            
            % Define the time grid used for the animation
            t = [obj.simulation.t_out(1):obj.rate:obj.simulation.t_out(end)];
            nt = size(t,2);
            
            % Interpolate the results from the solver to this time grid:
            x    = interp1(obj.simulation.t_out,obj.simulation.x_out.',t.').';

            % Fit animation graphics
            multiplier = (max(x(1, :)) - min(x(1, :))) / 8;

            axis([min(x(1, :)) - multiplier, ...
                  max(x(1, :)) + multiplier, ...
                  min(x(2, :)) - multiplier, ...
                  max(x(2, :)) + multiplier]);

            % Run Animation
            tic
            for i = 1:nt
                if i == 1
                    obj = obj.Update(t(:,i), x(:,i), 'init');
                else
                    obj = obj.Update(t(:,i), x(:,i), 'update');
                end
                while toc<t(:,i)*obj.slowDown
                    pause(0.01);
                end
            end
        end
        
        function obj = Update(obj, t, state, mode) 
            if isa(obj.model,'Mdl_TractorTrailerCLASS')
                plot(obj.trajectory.x(1,:), obj.trajectory.x(2,:), '--', 'Color', obj.grey, 'linewidth', 1.5), grid on,
            else
                plot(obj.trajectory.x(1,:), obj.trajectory.x(2,:), '--', 'Color', obj.grey, 'linewidth', 1.5), grid on,
            end

            for index=1:length(obj.model.graphics)

                x = state(obj.model.graphics(index).x);

                y = state(obj.model.graphics(index).y);
        
                theta = state(obj.model.graphics(index).theta);  
    
                color = obj.model.graphics(index).color;
    
                chassis_x = [x-obj.model.distance*3/4 , x+obj.model.distance*3/4, x+obj.model.distance*3/4, x-obj.model.distance*3/4];
    
                chassis_y = [y+obj.model.distance/2 , y+obj.model.distance/2 , y-obj.model.distance/2, y-obj.model.distance/2];
                
                if strcmp(mode, "init")
                    obj.robots(index) = fill(chassis_x, chassis_y, color);
                else
                    set(obj.robots(index),'XData',chassis_x,'YData',chassis_y);
                end 
                
                rotate(obj.robots(index), [0,0,1], theta*180/pi, [x, y, 0]);
            end

            drawnow;

            frame = getframe(1);

            im = frame2im(frame);

            [imind, cm] = rgb2ind(im,256);

            if strcmp(mode, "init")
                imwrite(imind, cm, 'test.gif', 'gif', 'DelayTime', 0, 'loopcount', inf);
            else
                imwrite(imind, cm, 'test.gif', 'gif', 'DelayTime', 0, 'writemode', 'append');
            end 
        end

        function Plot(obj)
            % Bring window to front:
            figure('Name', '2D-Output');
            if isa(obj.model,'Mdl_TractorTrailerCLASS')
                plot(obj.trajectory.x(4,:), obj.trajectory.x(5,:), '--', 'Color', obj.grey, 'linewidth', 1.5), grid on, hold on,
            else
                plot(obj.trajectory.x(1,:), obj.trajectory.x(2,:), '--', 'Color', obj.grey, 'linewidth', 1.5), grid on, hold on,
            end
            plot(obj.simulation.y_out(1,:), obj.simulation.y_out(2,:), '--', 'Color', obj.red, 'linewidth', 1.5), grid on, hold on,
            
            plot(obj.simulation.x_out(1,:), obj.simulation.x_out(2,:), '--', 'Color', obj.green, 'linewidth', 1.5), grid on, hold on,
                        
            if isa(obj.model,'Mdl_TractorTrailerCLASS')
                plot(obj.simulation.x_out(4,:), obj.simulation.x_out(5,:), '-', 'Color', obj.blue, 'linewidth', 1.5)
            end
            
            if obj.debug
                figure('Name','Error (x)');
                error = obj.simulation.x_out(4,:) - obj.trajectory.x(1,:); 
                plot(obj.trajectory.t(:), error(:), '-', 'Color', obj.blue, 'linewidth', 1.5), grid on, hold on,
    
                figure('Name','Error (y)');
                error = obj.simulation.x_out(5,:) - obj.trajectory.x(2,:); 
                plot(obj.trajectory.t(:), error(:), '-', 'Color', obj.blue, 'linewidth', 1.5), grid on, hold on,
    
                figure('Name','Error (theta)');
                error = obj.simulation.x_out(6,:) - obj.trajectory.x(3,:); 
                plot(obj.trajectory.t(:), error(:), '-', 'Color', obj.blue, 'linewidth', 1.5), grid on, hold on,
            end
        end
    end
end

