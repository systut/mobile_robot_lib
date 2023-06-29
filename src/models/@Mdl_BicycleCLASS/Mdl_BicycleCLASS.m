 classdef Mdl_BicycleCLASS
        properties
            % Model parameters
            length_base = 2;        % wheelbase length
            w_max = 3.14;           % maximum turning rate
                
            % Parameter variables
            p;

            % Number of states
            nx;
    
            % Number of input
            nu;            
        end
        
        methods
            function obj = Mdl_BicycleCLASS()
                %6DMODEL Construct an instance of this class
                %   Detailed explanation goes here                
                obj.nx = 4;
                
                obj.nu = 2;

                obj.p = [obj.length_base];
            end
        end

        methods(Static)
            SymbolicComputationOfEoM()
        end
end

