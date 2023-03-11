classdef Mdl_BicycleCLASS
    %6DMODEL Summary of this class goes here
    %   Detailed explanation goes here
    
    properties
        %% States
        nq;         % Number of Dofs
        %% Dimensions
        L = 2
        lr = 1.2
        w_max = 1.22
    end
    
    methods
        function obj = Mdl_BicycleCLASS()
            %6DMODEL Construct an instance of this class
            %   Detailed explanation goes here
            
            
        end

        function obj = step(obj, v, w)
            %STEP Construct an instance of this class
            %   Detailed explanation goes here
            if w > 0
                w = min(w, obj.w_max);
            else
                w = max(w, -obj.w_max);
            end

            obj.xc = obj.xc + v * np.cos(obj.theta + obj.beta) * obj.sample_time;
            obj.yc = obj.yc + v * np.sin(obj.theta + obj.beta) * obj.sample_time;
            obj.theta = obj.theta + v * np.tan(obj.delta) * np.cos(obj.beta) * obj.sample_time / obj.L;
            obj.delta = obj.delta + w * obj.sample_time;
            obj.beta = np.arctan(obj.lr * np.tan(obj.delta) / obj.L);
        end 

    end

    
end

