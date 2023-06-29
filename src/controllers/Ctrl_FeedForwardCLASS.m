classdef Ctrl_FeedForwardCLASS
    %CTRL_BASECLASS Summary of this class goes here
    %   Detailed explanation goes here
    
    properties
        model
    end
    
    methods
        function obj = Ctrl_FeedForwardCLASS(model)
            %CTRL_BASECLASS Construct an instance of this class
            %   Detailed explanation goes here
            obj.model = model;
        end
        
        function obj = Init(obj)
        end

        function [status, u, obj] = Loop(obj, reference_x, reference_u, y, index)
            %METHOD1 Summary of this method goes here
            %   Detailed explanation goes here
            status = true;
       
            u = reference_u(:, index-1);
        end
    end
end

