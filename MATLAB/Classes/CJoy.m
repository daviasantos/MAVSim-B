%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% CJoy
% Description: Joystick class. It opens, read and generate control commands 
% from the joystick. It considers a IPEGA PG 9076
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Author:              Prof Dr Davi A Santos (ITA)
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

classdef CJoy
    
    properties
        
        handle         % joystick handle
        a              % axes
        b              % buttons
        vx             % speed command along x
        vy             % speed command along y
        vz             % speed command along z
        wz             % speed command around z
        vxmax          % maximum speed in x
        vymax          % maximum speed in y
        vzmax          % maximum speed in z
        wzmax          % maximum speed around z
        
        
        
    end
    
    
    
    methods
    
        function obj = CJoy( sJoy )
        
            obj.vxmax = sJoy.vxmax;
            obj.vymax = sJoy.vymax;
            obj.vzmax = sJoy.vzmax;
            obj.wzmax = sJoy.wzmax;
           
        end
        
        %% Create a handle
        
        function obj = jopen( obj )
            
            obj.handle = vrjoystick(1);
        
        end
        
        
        %% Read the joystick axes and buttons
        
        function obj = jread( obj )
        
            % read axes 1-4 and buttons 5/6
    
            obj.a = axis( obj.handle,[1 2 3 4] );
            obj.b = button( obj.handle, [7 8] ); 
        
       
        end
        
        
        %% Generate the control commands
        
        function obj = jcommand( obj )
        
            % compute commands
    
            obj.vx = -obj.vxmax*obj.a(4);                     % in m/s
            obj.vy = -obj.vymax*obj.a(3);                     % in m/s
            obj.vz = -obj.vzmax*obj.a(2);                     % in m/s
            obj.wz = -obj.wzmax*obj.b(2)+obj.wzmax*obj.b(1);  % in deg/s
       
        
            % implement a dead zone

        
            
            if obj.vx < 0.1 && obj.vx > -0.1, obj.vx = 0; end
            if obj.vy < 0.1 && obj.vy > -0.1, obj.vy = 0; end
            if obj.vz < 0.1 && obj.vz > -0.1, obj.vz = 0; end
            if obj.wz < 0.1 && obj.wz > -0.1, obj.wz = 0; end
        
       
        end
        
        
            
    end
    
end

