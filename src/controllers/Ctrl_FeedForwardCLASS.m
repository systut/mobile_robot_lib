classdef Ctrl_FeedForwardCLASS
    %CTRL_BASECLASS Summary of this class goes here
    %   Detailed explanation goes here
    
    properties
        model
        trajectory
    end
    
    methods
        function obj = Ctrl_FeedForwardCLASS(model, trajectory)
            %CTRL_BASECLASS Construct an instance of this class
            %   Detailed explanation goes here
            obj.model = model;
            obj.trajectory = trajectory;
        end
        
        function obj = Init(obj)
        end

        function [status, u, obj] = Loop(obj, y, u, index)
            %METHOD1 Summary of this method goes here
            %   Detailed explanation goes here
            status = true;
       
            u = obj.trajectory.u(:, index-1);
        end
    end
end

