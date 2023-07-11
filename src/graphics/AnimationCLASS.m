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
            set(obj.fig,'Name','2D-Output');
            set(obj.fig,'Color','w');
            obj.debug = false;
        end

        function obj = Animate(obj)
            % Bring window to front:
            figure(obj.fig);
            plot(obj.trajectory.x(4,:), obj.trajectory.x(5,:), '--', 'Color', obj.grey, 'linewidth', 1.5), grid on, hold on,

            plot(obj.simulation.y_out(1,:), obj.simulation.y_out(2,:), '--', 'Color', obj.red, 'linewidth', 1.5), grid on, hold on,
            
            plot(obj.simulation.x_out(1,:), obj.simulation.x_out(2,:), '--', 'Color', obj.green, 'linewidth', 1.5)
            
            eps = 10;
            
            if obj.debug
                quiver(obj.trajectory.x(1,:), obj.trajectory.x(2,:), obj.trajectory.x(1,:) + eps * cos(obj.trajectory.x(3,:)), obj.trajectory.x(2,:) + eps * sin(obj.trajectory.x(3,:)),'-','filled')
            end

            if isa(obj.model,'Mdl_TractorTrailerCLASS')
                plot(obj.simulation.x_out(4,:), obj.simulation.x_out(5,:), '-', 'Color', obj.blue, 'linewidth', 1.5)
            end

            current_xlim = xlim;
            current_ylim = ylim;
            xlim([current_xlim(1)-0.2*(current_xlim(2)-current_xlim(1)), current_xlim(2)+0.2*(current_xlim(2)-current_xlim(1))]);
            ylim([current_ylim(1)-0.2*(current_ylim(2)-current_ylim(1)), current_ylim(2)+0.2*(current_ylim(2)-current_ylim(1))]);

            figure("Name","Output")

            subplot(3,1,1);

            plot(obj.simulation.t_out(1,:), obj.simulation.u_out(1,:), '-', 'Color', obj.blue, 'linewidth', 1.5),grid on, hold on
            title("vl");
            plot(obj.simulation.t_out(1,:), obj.trajectory.u(1,:), '--', 'Color', obj.green, 'linewidth', 1.5)

            subplot(3,1,2);

            plot(obj.simulation.t_out(1,:), obj.simulation.u_out(2,:), '-', 'Color', obj.blue, 'linewidth', 1.5),grid on, hold on
            title("vr");
            plot(obj.simulation.t_out(1,:), obj.trajectory.u(2,:), '--', 'Color', obj.green, 'linewidth', 1.5)

            subplot(3,1,3);
            plot(obj.simulation.t_out(1,:), (obj.simulation.x_out(6,:)-obj.simulation.x_out(3,:))/pi*180, '-', 'Color', obj.blue, 'linewidth', 1.5),grid on, hold on
            title("gamma");
            plot(obj.simulation.t_out(1,:), (obj.trajectory.x_out(6,:)-obj.trajectory.x_out(3,:))/pi*180, '--', 'Color', obj.green, 'linewidth', 1.5)
            
        end
    end
end

