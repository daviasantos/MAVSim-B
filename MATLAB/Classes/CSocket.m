%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% CSocket
% Description: Communication class. It implements a TCP sockets with 
%              with MATLAB assuming client role.
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Author:  Prof Dr Davi A Santos (ITA)
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%


classdef CSocket
    
    properties
         
        ip             % IP address
        port           % selected port
        handle         % handle for the socket object
        role           % 'client' or 'server'
        tout           % time out
        m_in           % message coming from outside
        m_out          % message to send
        nele           % maximum number of elements to read
        border         % byteOrder property of handle
        
    end
    
    
    methods
        
       
        %% Constructor
        
        function obj = CSocket ( sSocket )
            
            obj.ip     = sSocket.ip;
            obj.port   = sSocket.port;
            obj.tout   = sSocket.tout;
            obj.role   = sSocket.role;
            obj.nele   = sSocket.nele;
            obj.border = sSocket.border;
            
            
        end
        
        
        
        %% Initialize socket
        
        function obj = InitSocket( obj )
        
            obj.handle = tcpip( obj.ip,obj.port,'NetworkRole',obj.role );
     
            set( obj.handle,'Timeout',obj.tout );
            
            fopen( obj.handle ); 
            
            obj.handle.byteOrder = obj.border;
                
        end
        
        
        %% read TCP socket
        
        function obj = sread( obj )
 
            obj.m_in = char( fread( obj.handle, obj.nele ) );
        
        end
        
        
        %% write on TCP socket
        
        function swrite( obj )
        
            fwrite( obj.handle, obj.m_out,'single' );
  
        end
        
        
        %% close TCP socket
        
        function sclose( obj )
         
            fclose( obj.handle );
        
        end
        
        
            
        
    end
    
    
end

    