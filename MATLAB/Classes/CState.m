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
% CState
% Description: state machine class. 
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Author:              Prof Dr Davi A Santos (ITA)
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

classdef CState
   
    properties
        
        state
        event
        dt
        
    end
    
    properties (Constant) 
        
        % States
        
        OFF      = 1
        INIT     = 2
        READY    = 3
        ARMED    = 4
        TAKEOFF  = 5
        MANUAL   = 6
        WAYPOINT = 7
        LANDING  = 8
        
        % Transitions
        
        TURN_ON      = 1
        TURN_OFF     = 2
        INIT_END     = 3
        ARM_CMD      = 4
        DISARM_CMD   = 5
        TAKEOFF_CMD  = 6
        TAKEOFF_END  = 7
        LAND_CMD     = 8
        LAND_END     = 9
        WAYPOINT_CMD = 10 
        WAYPOINT_END = 11
        
    
    end
    
    methods
        
        function obj = CState ( ~ )
            
            obj.state = obj.OFF;
            obj.dt = 0;
            
        end
        
        function obj = StateTransition ( obj )
            
            switch obj.state
                
                case obj.OFF
                    
                    if obj.event == obj.TURN_ON
                        
                        obj.state = obj.INIT;
                        
                        obj.dt = 0;
                        
                        disp('Initialization...');
                        disp(' ');
                       
                    end
                
                case obj.INIT
                    
                    if obj.event == obj.INIT_END
                        
                        obj.state = obj.READY;
                       
                        obj.dt = 0;
                        
                        disp('Ready...');
                        disp(' ');
                        
                    end
                    
                case obj.READY
                    
                    if obj.event == obj.TURN_OFF
                        
                        obj.state = obj.OFF;
                        
                        obj.dt = 0;
                        
                        disp('Off.');
                        disp(' ');
                        
                        
                    elseif obj.event == obj.ARM_CMD
                        
                        obj.state = obj.ARMED;
                        
                        obj.dt = 0;
                        
                        disp('Armed...');
                        disp(' ');
                        
                    end
                    
                case obj.ARMED
                    
                    if obj.event == obj.DISARM_CMD
                        
                        obj.state = obj.READY;
                        
                        obj.dt = 0;
                        
                        disp('Ready...');
                        disp(' ');
                        
                        
                    elseif obj.event == obj.TAKEOFF_CMD
                        
                        obj.state = obj.TAKEOFF;
                        
                        obj.dt = 0;
                        
                        disp('Auto take off...');
                        disp(' ');
                        
                        
                    end
                    
                case obj.TAKEOFF
                    
                    if obj.event == obj.TAKEOFF_END
                        
                        obj.state = obj.MANUAL;
                        
                        obj.dt = 0;
                        
                        disp('Manual...');
                        disp(' ');
                        
                    end    
                
                case obj.MANUAL
                    
                    if obj.event == obj.LAND_CMD
                        
                        obj.state = obj.LANDING;
                        
                        obj.dt = 0;
                        
                        disp('Auto landing...');
                        disp(' ');
                        
                        
                    elseif obj.event == obj.WAYPOINT_CMD
                        
                        obj.state = obj.WAYPOINT;
                        
                        obj.dt = 0;
                        
                        disp('Auto waypoint-based guidance...');
                        disp(' ');
                        
                        
                    end
                    
                case obj.WAYPOINT
                    
                    if obj.event == obj.WAYPOINT_END
                        
                        obj.state = obj.MANUAL;
                        
                        obj.dt = 0;
                        
                        disp('Manual...');
                        disp(' ');
                        
                    end    
                    
                case obj.LANDING
                    
                    if obj.event == obj.LAND_END
                        
                        obj.state = obj.ARMED;
                        
                        obj.dt = 0;
                        
                        disp('Armed...');
                        disp(' ');
                        
                        
                    end   
                    
                    
                otherwise
                    % do nothing
                    
            end
            
                    
        end
        
        
        function obj = StateTime ( obj,Ts )
        
            obj.dt = obj.dt + Ts;
            
        end
        
        
    end
    
        
        
end


