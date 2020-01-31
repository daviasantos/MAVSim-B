%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% MAVSim: Simulation of an MAV dynamics & flight control 
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Description: pure matlab simulation. This version has a simple navigation
% algorithm (attitude determ. filter using mag, gyro, and acc + GPS). Here
% one can select manual (Joystick) or auto mode. 
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Author: Davi A. Santos (ITA)
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%% Add to path

addpath('Plots');
addpath('Classes');
addpath('Conversions');


%% Clean and close

clc
clear
close all

tstart = tic;

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% Parameters

% Input parameters

Parameters;


% Object: TCP socket with unity computer %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

sSocket_unity.ip   = ip_unity;
sSocket_unity.port = port_unity;
sSocket_unity.tout = tout_unity;
sSocket_unity.role = role_unity;
sSocket_unity.nele = nele_unity;

oSocket_unity    = CSocket( sSocket_unity );

% MAV %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

sMav.nr   = nr;
sMav.kf   = kf;
sMav.kt   = kt;
sMav.wmax = wmax;
sMav.km   = km;
sMav.Tm   = Tm;
sMav.l1   = l1;
sMav.l2   = l2;
sMav.m    = m;
sMav.JB   = JB;
sMav.Jr   = Jr;
sMav.g    = g;
sMav.h    = Ts;
sMav.w    = zeros(nr,1);
sMav.r    = zeros(3,1);
sMav.v    = zeros(3,1);
sMav.vp   = zeros(3,1);
sMav.D    = eye(3);
sMav.W    = zeros(3,1);


oMav = CMav( sMav );




% Disturbance/Uncertainty %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%


sUncer.alpha_f = alpha_f;
sUncer.alpha_t = alpha_t;


oUncer = CUncer( sUncer );



% Sensor platform %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%


sSensors.ba  = ba0;
sSensors.bg  = bg0;
sSensors.bm  = bm0;
sSensors.g   = g;
sSensors.mg  = mg;   
sSensors.sa  = sa;
sSensors.sg  = sg;
sSensors.sm  = sm;
sSensors.sba = sba;
sSensors.sbg = sbg;
sSensors.sbm = sbm;
sSensors.sr  = sr;
sSensors.Ts  = Ts;
sSensors.ya  = [0;0;g];
sSensors.yg  = zeros(3,1);
sSensors.ym  = mg;
sSensors.yr  = zeros(3,1); 
sSensors.yrp = zeros(3,1);


oSensors = CSensors( sSensors );




% Joystick %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

sJoy.vxmax = vxmax;
sJoy.vymax = vymax;
sJoy.vzmax = vzmax;
sJoy.wzmax = wzmax;

oJoy = CJoy( sJoy );




% Flight Control %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

sControl.K1      = K1;
sControl.K2      = K2;
sControl.K3      = K3;
sControl.K4      = K4;
sControl.Kc      = Kc;
sControl.JB      = JB;
sControl.Jr      = Jr;
sControl.m       = m;
sControl.g       = g;
sControl.nr      = nr;
sControl.l1      = l1;
sControl.l2      = l2;
sControl.kf      = kf;
sControl.kt      = kt;
sControl.k       = kt/kf;
sControl.Tmin    = Tmin;
sControl.Tmax    = Tmax;
sControl.Fmin    = Fmin;
sControl.Fmax    = Fmax;
sControl.zetamin = zetamin;
sControl.zetamax = zetamax;
sControl.r_      = zeros(3,1);
sControl.v_      = zeros(3,1);
sControl.p_      = 0;
sControl.wz_     = 0;
sControl.w_      = zeros(nr,1);

oControl = CControl( sControl );







% Trajectory planning/guidance %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%


sGuidance.wl    = wl;
sGuidance.Kpr   = Kpr;
sGuidance.Kpp   = Kpp;
sGuidance.Kdr   = Kdr;
sGuidance.Kdp   = Kdp;
sGuidance.rhor  = rhor;
sGuidance.rhop  = rhop;
sGuidance.dtl   = dtl;
sGuidance.Ts    = Ts;
sGuidance.dkl   = dtl/Ts;
sGuidance.nw    = size(wl,2);
sGuidance.l     = 1;
sGuidance.k     = 0;
sGuidance.r_    = zeros(3,1);
sGuidance.v_    = zeros(3,1);
sGuidance.p_    = 0;
sGuidance.wz_   = 0;
sGuidance.flag  = 0;


oGuidance = CGuidance( sGuidance );




% Navigation algorithm

sNavigation.tau = tau;
sNavigation.Ts  = Ts;
sNavigation.Ra  = Ra;
sNavigation.Rg  = Rg;
sNavigation.Rm  = Rm;
sNavigation.Rr  = Rr;
sNavigation.Qbg = Qbg;
sNavigation.mg  = mg;
sNavigation.x0  = x0;
sNavigation.P0  = P0;
            
oNavigation = CNavigation( sNavigation );








%% Initial interface

disp('%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%');
disp('MAVSim: SIMULATION OF AN MAV - DYNAMICS AND FLIGHT CONTROL');
disp(' ');
disp('Description: Pure MATLAB simulation including:'); 
disp(' ');
disp('             - plant and environment physics');
disp('             - sensors');
disp('             - guidance');
disp('             - flight control laws');
disp('             - attitude estimation (using mag, acc, gyro) and gps');
disp('             - manual and auto modes');
disp(' ');
disp('%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%');
disp('Author: Prof Dr Davi A. Santos');
disp('Institution: Aeronautics Institute of Technology (ITA/Brazil)');
disp('Last modification: January 29th, 2020');
disp('%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%');
disp(' ');


%% Mode selection


mode = -1;
while (mode ~= 0 && mode ~= 1)
    mode = input('Enter the desired flight mode ( 0 - manual; 1 - auto ): ');
    disp(' ');
    if mode == 1
        disp('Autonomous mode selected ...');
    elseif mode == 0
        disp('Joystick mode selected ...');
    end
end
disp(' ');



%% Connections

% Create joystick handle

oJoy = jopen( oJoy );


% Unity TCP socket setup and initialization

oSocket_unity = InitSocket( oSocket_unity );


%% Calibration

disp('Calibration...');
disp(' ');


for k = 1:kfcalib 
    
    % Sensor measurements
    
    oSensors.vp = oMav.vp;
    oSensors.W  = oMav.W;
    oSensors.r  = oMav.r;
    oSensors.D  = oMav.D;
    
    oSensors = acc ( oSensors );
    oSensors = gyro( oSensors );
    oSensors = mag ( oSensors );
    oSensors = gps ( oSensors );
    
    
    % Navigation
    
    oNavigation.ya  = oSensors.ya;
    oNavigation.yg  = oSensors.yg;
    oNavigation.ym  = oSensors.ym;
    oNavigation.yr  = oSensors.yr;
    oNavigation.yrp = oSensors.yrp;
    
    
    oNavigation = NV( oNavigation );
    
    
end


%% Discrete-time loop

disp('Simulation loop running...');
disp(' ');


% Initialize time index

k = 1;  


while( 1 )
    
    t1 = tic;
    
    %% Compute commands (manual mode) - joystick
    
    if ~mode 
        
        oJoy = jread( oJoy );
        oJoy = jcommand( oJoy );
        
        oControl.v_   = [oJoy.vx,oJoy.vy,oJoy.vz]';
        oControl.wz_  =  oJoy.wz;
        
        oControl.r_   =  oControl.r_ + oControl.v_*Ts;
        oControl.p_   =  oControl.p_ + oControl.wz_*Ts;
      
    else 
        
        
        
    %% Compute commands (auto mode) - Guidance

        % input measurement

        oGuidance.r  = oNavigation.x(1:3); 
        oGuidance.v  = oNavigation.x(4:6); 
        aux          = D2a( q2D( oNavigation.x(10:13 )) );  
        oGuidance.p  = aux(3); 
        oGuidance.wz = oSensors.yg(3) - oNavigation.x(16); 
        
        
        % wayset verification

        oGuidance = wayset_verif( oGuidance );

        % wayset transition

        oGuidance = wayset_transit( oGuidance ); 

        % command generation

        oGuidance = plaw( oGuidance );

      
        % Commands for flight control 
        
        oControl.r_    = oGuidance.r_;
        oControl.v_    = oGuidance.v_;
        oControl.p_    = oGuidance.p_; 
        oControl.wz_   = oGuidance.wz_;
    
        
    end 
    
    
    
 
    %% Flight control
    
    
    % input variables 
    
    oControl.r     = oNavigation.x(1:3); 
    oControl.v     = oNavigation.x(4:6); 
    oControl.D     = q2D( oNavigation.x(10:13) );  
    oControl.W     = oSensors.yg - oNavigation.x(14:16);   
    
    
    % Position or velocity control law

    if mode
        oControl = PC( oControl );
    else
        oControl = VC( oControl );
    end
    
    % Attitude command computation
    
    oControl = ATC( oControl );
    
    % Attitude control law
    
    oControl = AC( oControl );
    
    % Control allocation algorithm
    
    oControl = CA( oControl );
    
    
    
    %% Environment, plant, and sensor platform simulation
    
    % Disturbances
    
    oUncer  = disturbances( oUncer );
    
    
    % Equations of motion
    
    oMav.Fd = oUncer.Fd;
    oMav.Td = oUncer.Td;
    oMav.w_ = oControl.w_;
    
    oMav = propeller( oMav );
    oMav = efforts  ( oMav );
    oMav = dynamics ( oMav );
    
    
    % Sensor platform
    
    oSensors.vp = oMav.vp;
    oSensors.W  = oMav.W;
    oSensors.r  = oMav.r;
    oSensors.D  = oMav.D;
    
    oSensors = acc ( oSensors );
    oSensors = gyro( oSensors );
    oSensors = mag ( oSensors );
    oSensors = gps ( oSensors );
    

    %% Navigation algorithm
   
    
    oNavigation.ya  = oSensors.ya;
    oNavigation.yg  = oSensors.yg;
    oNavigation.ym  = oSensors.ym;
    oNavigation.yr  = oSensors.yr;
    oNavigation.yrp = oSensors.yrp;
    
    oNavigation = NV( oNavigation );
    
    
    %% Animation
    
    % Initialize data members with true pose and power
    
    oData.r = oMav.r;
    oData.a = 180/pi*D2a( oMav.D );
    oData.power = 1;
    
    % Pack and write data to Unity
    
    oData               = PackUnity( oData );
    oSocket_unity.m_out = oData.m_out_unity;
    swrite( oSocket_unity );
    
    
    
    %% Monitoring 
    
    % Estimates
    
    red(:,k)  = oNavigation.x(1:3);
    ved(:,k)  = oNavigation.x(4:6);
    baed(:,k) = oNavigation.x(7:9);
    bged(:,k) = oNavigation.x(14:16);
    aed(:,k)  = 180/pi*D2a( q2D( oNavigation.x(10:13) ) );
  
    
    % True states
    
    rd(:,k)   = oMav.r; 
    vd(:,k)   = oMav.v;
    Wd(:,k)   = oMav.W;
    ad(:,k)   = 180/pi*D2a( oMav.D );
    
    % commands 
    
    rd_(:,k)  = oControl.r_;
    ad_(:,k)  = 180/pi*D2a( oControl.D_ );
    FGd(:,k)  = oControl.FG_;
    TBd(:,k)  = oControl.TB_;
    
    
    %% Waiting for sampling instant (manual mode)
    
    t2 = toc(t1);
    
    if ~mode
        dt = uint64(1000*(Ts-t2)); 
        java.lang.Thread.sleep(dt);
    end
    
    
    %% Update time index (just for plotting)
    
    k = k + 1;
    if k == tf/Ts
        break;
    end
    
      
end 


%% Simulation time

tend = toc(tstart);
disp('Simulation finished! Duration:');
disp(' ');
disp(tend);
    

%% Plots

disp('Plotting...');
disp(' ');

t = 2*Ts:Ts:tf;

% position vs position command

plot1c(t,rd(1,:)','$r_1$',rd_(1,:)','$\bar{r}_1$');
plot1c(t,rd(2,:)','$r_2$',rd_(2,:)','$\bar{r}_2$');
plot1c(t,rd(3,:)','$r_3$',rd_(3,:)','$\bar{r}_3$');

% attitude vs attitude command

plot1c(t,ad(1,:)','$a_1$',ad_(1,:)','$\bar{a}_1$');
plot1c(t,ad(2,:)','$a_2$',ad_(2,:)','$\bar{a}_2$');
plot1c(t,ad(3,:)','$a_3$',ad_(3,:)','$\bar{a}_3$');


% force command 

plot3c(t,FGd','$F_1$','$F_2$','$F_3$');


% torque command

plot3c(t,TBd','$T_1$','$T_2$','$T_3$');


% 3D trajectory

plot3dc(rd,rd_,'Trajectory','Command');
 
   
% for evaluating/tuning the filters

plotEvalNavigation;



