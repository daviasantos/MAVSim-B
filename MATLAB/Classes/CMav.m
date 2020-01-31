%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% CMAV
% Description: MAV physics class. It implements the physics of the MAV under
% consideration. It uses RK4 to integrate the equations of motion. 
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Author:              Prof Dr Davi A Santos (ITA)
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

classdef CMav
   
    
    properties
        
        % physical parameters
        
        nr       % number of rotors
        kf       % thrust coefficient
        kt       % torque coefficient
        wmax     % maximum rotation speed of the rotors
        fmax     % maximum thrust of a rotor
        km       % motor coefficient
        Tm       % motor time constant
        l1       % longitudinal arm length
        l2       % transverse arm length
        m        % mass
        JB       % inertia matrix
        JBi      % JB inverse
        Jr       % moment of inertia of the rotors      
        g        % gravity
        
        % simulation parameters
        
        h        % integration step
        
        % symbols
        
        e3
        
    
        % state variables
        
        r        % position
        v        % velocity 
        vp       % total acceleration
        D        % attitude (matrix)
        W        % angular velocity
        w        % rotor speeds 
        
        % command and inputs
        
        w_       % rotor speed commands
        Fd       % disturbance force
        Td       % disturbance torque
        
        % computed variables
        
        f        % thrust force
        k        % kt/kf
        F        % resulting force magnitude
        TB       % resulting torque in SB
        G        % allocation matrix
    
    
    end
    
    
    
    methods
        
        
        %% Constructor
        
        function obj = CMav( sMav )
            
            % Initialization
            
            obj.nr   = sMav.nr;
            obj.kf   = sMav.kf;
            obj.kt   = sMav.kt;
            obj.wmax = sMav.wmax;
            obj.km   = sMav.km;
            obj.Tm   = sMav.Tm;
            obj.l1   = sMav.l1;
            obj.l2   = sMav.l2;
            obj.m    = sMav.m;
            obj.JB   = sMav.JB;
            obj.Jr   = sMav.Jr;
            obj.g    = sMav.g;
            obj.h    = sMav.h;
            obj.w    = sMav.w;
            obj.r    = sMav.r;
            obj.v    = sMav.v;
            obj.vp   = sMav.vp;
            obj.D    = sMav.D;
            obj.W    = sMav.W;
            
            % Pre-computation:
            
            obj.e3 = [0;0;1];
            obj.fmax = obj.kf*obj.wmax^2;
            obj.k = obj.kt/obj.kf;
            obj.G = [1 1 1 1;
                     obj.l2 -obj.l2 -obj.l2 obj.l2;
                    -obj.l1 -obj.l1 obj.l1 obj.l1;
                     obj.k -obj.k obj.k -obj.k];      
            obj.JBi = inv( obj.JB );
            obj.f   = zeros(obj.nr,1);

            
        end
        
        
        
        
       
        %% Aerodynamics of the propellers
        
        function obj = propeller( obj )
       
            
            for i=1:obj.nr 
                obj.f(i) = obj.kf*obj.w(i)^2;
            end    
            
         
            for i=1:obj.nr
                if obj.f(i)<0, obj.f(i) = 0; end
                if obj.f(i)>obj.fmax, obj.f(i) = obj.fmax; end
            end
            
            
                    
        end
        

        %% Resulting efforts
        
        function obj = efforts( obj )
       
            V = obj.G*obj.f;
            
            obj.F  = V(1);
            obj.TB = V(2:4);
            
        end
        
        
        %% Equations of motion
        
        function obj = dynamics( obj )
        
            
            % state and input vectors
            
            q = D2q(obj.D);
            x = [obj.r;obj.v;q;obj.W;obj.w];
            
            % integration using HK4
            
            k1 = obj.h*fun( x );
            k2 = obj.h*fun( x+k1/2 );
            k3 = obj.h*fun( x+k2/2 );            
            k4 = obj.h*fun( x+k3 ); 
            x  = x + k1/6 + k2/3 + k3/3 + k4/6;
            
            % updated states
            
            obj.r = x(1:3);
            obj.v = x(4:6);
            obj.D = q2D(x(7:10));
            obj.W = x(11:13);
            obj.w = x(14:13+obj.nr);
            
      
            
            %% the function itself (it is a child)
            
            function xp = fun( x )
                   
                v1 = x(4:6);
                D1 = q2D(x(7:10));
                q1 = x(7:10);
                W1 = x(11:13);
                w1 = x(14:13+obj.nr);
                
                 
                
              
                xp = [v1;
                      (1/obj.m)*D1'*obj.e3*obj.F-obj.e3*obj.g+(1/obj.m)*obj.Fd;
                      1/2*[-cruz(W1),W1;-W1',0]*q1;
                      obj.JBi*(cruz(obj.JB*W1)*W1+(obj.TB + obj.Td));
                      -1/obj.Tm*w1+obj.km/obj.Tm*obj.w_];
                
                  
                obj.vp = xp(4:6);  
                
            end
            
            
        end
        
        
       
        
    end
    
end


