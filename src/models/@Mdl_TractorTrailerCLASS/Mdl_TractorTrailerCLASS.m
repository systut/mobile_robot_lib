classdef Mdl_TractorTrailerCLASS
    %6DMODEL Summary of this class goes here
    %   Detailed explanation goes here
    
    properties
        % Parameters of the model
        length_front = 0.01;

        length_back = 0.01;

        distance = 0.53/2;

        slip_right = 0;

        slip_left = 0;

        % Parameter variables
        p;

        % Number of states
        nx;

        % Number of input
        nu;
    end
    
    methods
        function obj = Mdl_TractorTrailerCLASS()
            %6DMODEL Construct an instance of this class
            %   Detailed explanation goes here            
            obj.nx = 6;

            obj.nu = 2;

            obj.p = [obj.length_front, obj.length_back, obj.slip_right, obj.slip_left, obj.distance];
        end
        A       = SystemMatrix(obj, state, input, dt, p);
        B       = ControlMatrix(obj, state, input, dt, p);
        F       = Function(obj, state, input, dt, p);
    end

    methods(Static)
        SymbolicComputationOfEoM() % Defined externally
    end
end

