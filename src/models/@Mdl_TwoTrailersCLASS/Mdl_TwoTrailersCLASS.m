classdef Mdl_TwoTrailersCLASS
    %MDL_TWOTRAILERSCLASS Summary of this class goes here
    %   Detailed explanation goes here
    
    properties

        % Model's specifications
        distance = 0.48;

        Lt1 = 0.25;

        L2 = 0.25;

        Lt2 = 0.25;

        L3 = 0.25;

        % Slip parameters
        slip_right = 0.0;

        slip_left = 0.0;
        % Generated MATLAB parameters
        p;

        p_without_slip;

        % Number of states
        nx;

        % Number f input
        nu;

        % Graphics information
        % graphics;
    end
    
    methods
        function obj = Mdl_TwoTrailersCLASS()
            obj.nx = 9;

            obj.nu = 2;

            obj.p_without_slip = [obj.distance, obj.Lt1, obj.L2, obj.Lt2, obj.L3, 0, 0];

            obj.p = [obj.distance, obj.Lt1, obj.L2, obj.Lt2, obj.L3, obj.slip_right, obj.slip_left];

        end
        
        A = SystemMatrix(obj, state, input, dt, p);
        B = ControlMatrix(obj, state, input, dt, p);
        F = Function(obj, state, input, dt,  p);
    end

    methods(Static)
        SymbolicComputationOfEoM()  % Definded externally
    end
end

