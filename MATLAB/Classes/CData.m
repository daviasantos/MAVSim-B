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
% CData
% Description: data manipulation class. It implements the function used to
%              pack data into a string message and another to unpack a
%              string message into the corresponding variables.
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Author: Prof Dr Davi A Santos (ITA)
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%


classdef CData
    
    properties
         
        % String messages
        
        m_in_embedded     % message coming from embedded computer
        m_out_embedded    % message going to embedded computer
        m_in_unity        % message coming for unity
        m_out_unity       % message going to unity

        % Sensor data
        
        ya                % acc measure
        yg                % gyro measure
        ym                % mag measure
        yr                % gps measure
        yrp               % gps rate measure 
        
        % Control commands
        
        w_                % speed commands to the rotors
        FG_               % force command 
        TB_               % torque command 
        a_                % attitude command (Euler angles 123)
        r_                % position command
        
        % Navigation state and cov
        
        xe                % state estimates
        pe                % variance estimates
        
        % true pose
        
        r                 % position
        a                 % attitude
        
        % flags
        
        power             % power on (if 1) or of (if 0)
        state             % from state machine to Unity
        
        
        % parameters
        
        nr                % number of rotors
        
    end
    
    
    
    
    methods
        
       
        %% Constructor
        
        function obj = CData( nr )
            
            obj.nr  = nr;
            
            obj.w_  = zeros( obj.nr,1 );
            obj.FG_ = zeros( 3,1 );
            obj.TB_ = zeros( 3,1 );
            obj.a_  = zeros( 3,1 );
            obj.r_  = zeros( 3,1 );
            obj.xe  = zeros( 16,1 );
            obj.pe  = zeros( 16,1 );
                   
        end
        
        
        
        %% Pack data to send to embedded computer
        
        function obj = PackEmbedded ( obj )

            obj.m_out_embedded = [];
            
            for i=1:3, obj.m_out_embedded = [obj.m_out_embedded,num2str(obj.ya(i),'%.7f'),' ']; end
            
            for i=1:3, obj.m_out_embedded = [obj.m_out_embedded,num2str(obj.yg(i),'%.7f'),' ']; end
            
            for i=1:3, obj.m_out_embedded = [obj.m_out_embedded,num2str(obj.ym(i),'%.7f'),' ']; end
         
            for i=1:3, obj.m_out_embedded = [obj.m_out_embedded,num2str(obj.yr(i),'%.7f'),' ']; end
         
            for i=1:3, obj.m_out_embedded = [obj.m_out_embedded,num2str(obj.yrp(i),'%.7f'),' ']; end
            

        end

        
        
        %% Unpack data coming from the embedded computer
        
        function obj = UnpackEmbedded ( obj )
           
            
            % w_
            
            i = 1;
            j = 1;
            
            for l = 1: obj.nr
                
                while ( obj.m_in_embedded(j) ~= ' ' )
                    s_in(i) = obj.m_in_embedded(j); 
                    i = i + 1;
                    j = j + 1;
                end
                obj.w_(l) = str2double(s_in(1:i-1));
            
                i = 1;
                j = j + 1;
            
            end
            
            
            % FG_
            
            for l = 1:3
                
                while ( obj.m_in_embedded(j) ~= ' ' )
                    s_in(i) = obj.m_in_embedded(j); 
                    i = i + 1;
                    j = j + 1;
                end
                obj.FG_(l) = str2double(s_in(1:i-1));

                i = 1;
                j = j + 1;
            
            end
            
            
            % TB_
         
            for l = 1:3
                
                while ( obj.m_in_embedded(j) ~= ' ' )
                    s_in(i) = obj.m_in_embedded(j); 
                    i = i + 1;
                    j = j + 1;
                end
                obj.TB_(l) = str2double(s_in(1:i-1));

                i = 1;
                j = j + 1;
                
            end
            
            
            % a_
            
            for l = 1:3
                
                while ( obj.m_in_embedded(j) ~= ' ' )
                    s_in(i) = obj.m_in_embedded(j); 
                    i = i + 1;
                    j = j + 1;
                end
                obj.a_(l) = str2double(s_in(1:i-1));

                i = 1;
                j = j + 1;

            end
            
            
            % r_ 
            
            for l = 1:3
                
                while ( obj.m_in_embedded(j) ~= ' ' )
                    s_in(i) = obj.m_in_embedded(j); 
                    i = i + 1;
                    j = j + 1;
                end
                obj.r_(l) = str2double(s_in(1:i-1));

                i = 1;
                j = j + 1;
                
            end
            
            
            % xe
            
            for l = 1:16
                
                while ( obj.m_in_embedded(j) ~= ' ' )
                    s_in(i) = obj.m_in_embedded(j); 
                    i = i + 1;
                    j = j + 1;
                end
                obj.xe(i) = str2double(s_in(1:i-1));


                i = 1;
                j = j + 1;
                
            end
            
            
            % pe
            
            for l = 1:16

                while ( obj.m_in_embedded(j) ~= ' ' )
                    s_in(i) = obj.m_in_embedded(j); 
                    i = i + 1;
                    j = j + 1;
                end
                obj.pe(l) = str2double(s_in(1:i-1));

                i = 1;
                j = j + 1;
            
            end
            
  

        end
        
        
        
        %% Pack data to send to Unity
        
        function obj = PackUnity ( obj )

            obj.m_out_unity = single([obj.r' obj.a' obj.power obj.state]);
           
        end


        %% Unpack data received from Unity
        
        function obj = UnpackUnity ( obj )

            
            

        end
        
        
        
            
        
    end
    
    
end

    