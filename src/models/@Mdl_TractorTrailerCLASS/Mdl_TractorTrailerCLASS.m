classdef Mdl_TractorTrailerCLASS
    %6DMODEL Summary of this class goes here
    %   Detailed explanation goes here
    
    properties
        % Parameters of the model
        length_front = 0.5;

        length_back = 0.5;

        distance = 0.48;

        slip_right = 0.0;

        slip_left = 0.0;

        % Parameter variables
        p;

        p_without_slip;

        % Number of states
        nx;

        % Number of input
        nu;
        
        % Graphics information
        graphics;
    end
    
    methods
        function obj = Mdl_TractorTrailerCLASS()
            %6DMODEL Construct an instance of this class
            %   Detailed explanation goes here            
            obj.nx = 6;

            obj.nu = 2;

            obj.p = [obj.length_front, obj.length_back, obj.slip_right, obj.slip_left, obj.distance];
        
            obj.p_without_slip = [obj.length_front, obj.length_back, 0.0, 0.0, obj.distance];

            obj.graphics = [struct('x', 1, 'y', 2, 'theta', 3, 'color', 'w'); ...
                            struct('x', 4, 'y', 5, 'theta', 6, 'color', 'r')];
        end
        A       = SystemMatrix(obj, state, input, dt, p);
        B       = ControlMatrix(obj, state, input, dt, p);
        F       = Function(obj, state, input, dt, p);
    end

    methods(Static)
        SymbolicComputationOfEoM() % Defined externally
    end
end

