
addpath('Plots');   

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


%% Update time index (just for plotting)
    
    k = k + 1;
    if k == tf/Ts
        break;
    end
    
    
    
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