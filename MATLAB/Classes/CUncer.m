%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% CUncer
% Description: Uncertainty generation class. It computes the disturbance
% torque and force.
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Author:              Prof Dr Davi A Santos (ITA)
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%


classdef CUncer
    
    properties
         
        alpha_f        % bound on disturbance force components (3,1)
        alpha_t        % bound on disturbance torque components (3,1)
        
        beta_f         % precomputation of 2*diag(alpha_f)
        beta_t         % precomputation of 2*diag(alpha_f)
     
        Fd             % disturbance force
        Td             % disturbance torque
        
        
    end
    
    
    methods
        
       
        %% Constructor
        
        function obj = CUncer( sUncer )
        
            % Initialization
            
            obj.alpha_f = sUncer.alpha_f;
            obj.alpha_t = sUncer.alpha_t;
            
            
            % Pre-computation
            
            obj.beta_f = 2*diag(obj.alpha_f);
            obj.beta_t = 2*diag(obj.alpha_t);
            
        end
        
        
        
        
        %% Generate the disturbance force and torque
        
        function obj = disturbances( obj )
        
            % using uniform distributed random vectors
            
            obj.Fd = -obj.alpha_f + obj.beta_f*rand(3,1);            
            obj.Td = -obj.alpha_t + obj.beta_t*rand(3,1);   
                  
            
        end
        
        
       
        
            
        
    end
    
    
end

    