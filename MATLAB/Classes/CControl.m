%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% CControl
% Description: Flight control class. It implements the flight control laws,
% and control allocation.
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Author: Prof Dr Davi A Santos (ITA)
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

classdef CControl
    
    properties
    
        % Control parameters
        
        K1              % prop gain attitude control
        K2              % deriv gain attitude control
        K3              % prop gain position control
        K4              % deriv gain position control
        Kc              % prop gain velocity control 
        JB              % inertia matrix
        Jr              % rotor inertia 
        m               % mass
        g               % gravity
        nr              % number of rotors
        l1              % frontal arm
        l2              % lateral arm
        kf              % force coefficient
        kt              % torque coefficient
        k               % kt/kf
        Ginv            % inverse of control alloc matrix
        Tmin            % minimum torque
        Tmax            % maximum torque
        Fmin            % minimum force
        Fmax            % maximum force
        zetamin         % minimum virtual thrust
        zetamax         % maximum virtual thrust

        % External variables
        
        r                % position 
        v                % velocity 
        D                % attitude matrix 
        W                % angular velocity 
        r_               % position command 
        v_               % velocity command 
        p_               % heading command 
        wz_              % heading rate command 
        w_               % speed commands
        cax              % accel command in x (joystick)
        cay              % accel command in y (joystick)


        % Internal variable

        D_               % attitude matrix command 
        ea               % attitude error (3D representation)
        f_               % thrust commands
        TB_              % torque command in SB 
        FG_              % force command in SG 
        nG_              % normal direction command 
        F_               % force magnitude command 
        
        % Symbols
        
        e3               % (0,0,1)
        
   
    end
    
    methods
        
        %% Constructor
        
        function obj = CControl ( sControl )
            
            % Initialization
            
            obj.K1      = sControl.K1;
            obj.K2      = sControl.K2;
            obj.K3      = sControl.K3;
            obj.K4      = sControl.K4;
            obj.Kc      = sControl.Kc;
            obj.JB      = sControl.JB;
            obj.Jr      = sControl.Jr;
            obj.m       = sControl.m;
            obj.g       = sControl.g;
            obj.nr      = sControl.nr;
            obj.l1      = sControl.l1;
            obj.l2      = sControl.l2;
            obj.kf      = sControl.kf;
            obj.kt      = sControl.kt;
            obj.k       = sControl.k;
            obj.Tmin    = sControl.Tmin;
            obj.Tmax    = sControl.Tmax;
            obj.Fmin    = sControl.Fmin;
            obj.Fmax    = sControl.Fmax;
            obj.zetamin = sControl.zetamin;
            obj.zetamax = sControl.zetamax;
            obj.r_      = sControl.r_;
            obj.v_      = sControl.v_;
            obj.p_      = sControl.p_;
            obj.wz_     = sControl.wz_;
            obj.w_      = sControl.w_;
            
            
            % Pre-computation
            
            obj.e3 = [0;0;1];
            obj.k = obj.kt/obj.kf;
            obj.Ginv = 0.25* [1 1/obj.l2 -1/obj.l1 1/obj.k;
                              1 -1/obj.l2 -1/obj.l1 -1/obj.k;
                              1 -1/obj.l2 1/obj.l1 1/obj.k;
                              1 1/obj.l2 1/obj.l1 -1/obj.k];
            
            
            
            
        end
        
        
        
        %% Position control
        
        function obj = PC( obj )
            
            % Control law ifself
            
            obj.FG_ = obj.m*( obj.g*obj.e3 + obj.K3*(obj.r_-obj.r) + obj.K4*(obj.v_-obj.v) );
            obj.FG_ = sat( obj.FG_, obj.Fmin, obj.Fmax );
           
            
            % Force magnitude and direction commands
            
            obj.F_  = norm( obj.FG_ );
            obj.nG_ = obj.FG_/obj.F_;
            
            
        end
        
        
        %% Velocity control
        
        function obj = VC( obj )
   
            % Control law ifself
            
            obj.FG_ = obj.m*( obj.g*obj.e3 + obj.Kc*(obj.v_-obj.v) );
            obj.FG_ = sat( obj.FG_, obj.Fmin, obj.Fmax );
           
            
            % Force magnitude and direction commands
            
            obj.F_  = norm( obj.FG_ );
            obj.nG_ = obj.FG_/obj.F_;
            
        end
        
        
        %% Attitude control
        
        function obj = AC( obj )

            % Attitude error
            
            obj.ea = D2a( obj.D_*obj.D' );
            
           
            %  Control law itself

            obj.TB_ = cruz(obj.W)*( obj.JB*obj.W ) + ... 
                  obj.JB*( obj.K1*obj.ea + obj.K2*([0 0 obj.wz_]'-obj.W) );
            obj.TB_ = sat( obj.TB_, obj.Tmin, obj.Tmax );    
            
   
        end
        
        
        %% Attitude command computation
        
        function obj = ATC( obj )
            
            % Convert from normal vector to phi-theta
            
            phi   = -atan(obj.nG_(2)/obj.nG_(3));
            theta =  asin(obj.nG_(1));
            psi   =  obj.p_;
            
            % Conversion to attitude matrix considering psi_      
            
            obj.D_ = a2D([phi theta psi]);

            
        end
        
        
        %% Attitude command computation for MANUAL state
        
        function obj = ATCman( obj )
            
            % Convert from normal vector to phi-theta
            
            phi1   = -atan(obj.nG_(2)/obj.nG_(3));
            theta1 =  asin(obj.nG_(1));
            
            % Convert joystick commands into attitude command
            
            phi2   = -obj.cay;
            theta2 =  obj.cax;
            psi   =  obj.p_;  
            
            % Conversion to attitude matrix considering psi_ 
            
            phi   = phi1 + phi2;
            theta = theta1 + theta2;
            
            obj.D_ = a2D([phi theta psi]);

            
        end
        
        
        %% Control allocation
        
        function obj = CA( obj )
                           
            % Thrust commands
            
            obj.f_ = obj.Ginv*[obj.F_;
                               obj.TB_];         

            obj.f_ = sat( obj.f_, obj.zetamin*ones(obj.nr,1), obj.zetamax*ones(obj.nr,1) );
            
            % Rotational speed commands
            
            for i=1:obj.nr
                
                obj.w_(i) = sqrt( obj.f_(i)/obj.kf );
            
            end
            
            
        end
        
        
        
    end
end

