classdef Mdl_DifferentialDriveCLASS
    %6DMODEL Summary of this class goes here
    %   Detailed explanation goes here
    
    properties
        %% State
        x;
        y;
        theta;

    end
    
    methods
        function obj = Mdl_DifferentialDriveCLASS()
            %6DMODEL Construct an instance of this class
            %   Detailed explanation goes here
            
            
        end
        function obj = Update(obj, u) 
            [x,y,z] = obj.model.Update(u);
        end
    end

    
end

