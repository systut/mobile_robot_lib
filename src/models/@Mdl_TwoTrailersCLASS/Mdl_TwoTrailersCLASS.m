classdef Mdl_TwoTrailersCLASS
    %MDL_TWOTRAILERSCLASS Summary of this class goes here
    %   Detailed explanation goes here
    
    properties

        % Model's specifications
        d = 0.48;

        Lt1 = 0.2;

        L2 = 0.3;

        Lt2 = 0.2;

        L3 = 0.3;

        % Slip parameters
        s_r = 0.0;

        s_l = 0.0;
        % Generated MATLAB parameters
        p_slip;

        p_no_slip;

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

            obj.p_no_slip = [obj.d, obj.Lt1, obj.L2, obj.Lt2, obj.L3, 0, 0];

            obj.p_slip = [obj.d, obj.Lt1, obj.L2, obj.Lt2, obj.L3, obj.s_r, obj.s_l];

        end
        
        A = SystemMatrix(obj, state, input, dt, p);
        B = ControlMatrix(obj, state, input, dt, p);
        F = Function(obj, state, input, dt,  p);
    end

    methods(Static)
        SymbolicComputationOfEoM()  % Definded externally
    end
end

