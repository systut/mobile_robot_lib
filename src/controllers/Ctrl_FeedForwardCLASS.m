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
        
        function u = Loop(obj, reference_state, reference_input)
            %METHOD1 Summary of this method goes here
            %   Detailed explanation goes here

            u = reference_input;
        end
    end
end

