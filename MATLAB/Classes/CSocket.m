%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%    MAVSim - Flight dynamics and control simulator for MAVs 
%    Copyright (C) 2020  Aeronautics Institute of Technology
%
%    This program is free software: you can redistribute it and/or modify
%    it under the terms of the GNU General Public License as published by
%    the Free Software Foundation, either version 3 of the License, or
%    (at your option) any later version.
%
%    This program is distributed in the hope that it will be useful,
%    but WITHOUT ANY WARRANTY; without even the implied warranty of
%    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
%    GNU General Public License for more details.
%
%    You should have received a copy of the GNU General Public License
%    along with this program. If not, see <https://www.gnu.org/licenses/>.
%
%    Also add information on how to contact you by electronic and paper mail.
%    To contact the author, please use the electronic address davists@ita.br or 
%    send a letter to
%    
%    Prof. Dr. Davi Antonio dos Santos
%    Divisao de Engenharia Mecanica
%    Instituto Tecnologico de Aeronautica
%    Praça Marechal Eduardo Gomes, 50, Vila das Acacias, 12228-900, Sao Jose dos Campos,
%    SP, Brasil.
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

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

    