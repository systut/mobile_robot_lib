classdef Obs_NormalCLASS
    %OBS_NORMALCLASS Summary of this class goes here
    %   Detailed explanation goes here
    
    properties
        noise_mu;

        noise_sigma;

        noise_variance;

        model;
    end
    
    methods
        function obj = Obs_NormalCLASS(model)
            %OBS_NORMALCLASS Construct an instance of this class
            %   Detailed explanation goes here
            rng('default');

            obj.model = model;

            obj.noise_mu = zeros(obj.model.nx,1);
        end
        
        function state_out = Observe(obj, state)
            %METHOD1 Summary of this method goes here
            %   Detailed explanation goes here
            obj.noise_variance = chol(obj.noise_sigma);

            measure_noise = randn(1, obj.model.nx) * obj.noise_variance;

            measure_noise = obj.noise_mu + measure_noise';

            state_out = state;
        end
    end
end

