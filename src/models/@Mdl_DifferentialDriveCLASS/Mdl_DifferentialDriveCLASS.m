classdef Mdl_DifferentialDriveCLASS
    %6DMODEL Summary of this class goes here
    %   Detailed explanation goes here
    
    properties
        % Number of states
        nx;

        % Number of input
        nu;
        
        % Parameters
        slip_right = 0.5;

        slip_left = 0.5;

        distance = 0.53/2; 

        % Parameter vector
        p;          

        p_without_slip;
    end
    
    methods
        function obj = Mdl_DifferentialDriveCLASS()
            %6DMODEL Construct an instance of this class
            %   Detailed explanation goes here            
            obj.nx = 3;

            obj.nu = 2;

            obj.p = [obj.slip_right, obj.slip_left, obj.distance];

            obj.p_without_slip = [0, 0, obj.distance];
        end
        A       = SystemMatrix(obj, state, input, dt, p);
        B       = ControlMatrix(obj, state, input, dt, p);
        F       = Function(obj, state, input, dt, p);
    end

    methods(Static)
        SymbolicComputationOfEoM()
    end
end

