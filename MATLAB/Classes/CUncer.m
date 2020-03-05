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
% CUncer
% Description: Uncertainty generation class. It computes the disturbance
% torque and force.
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Author: Prof Dr Davi A Santos (ITA)
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

    