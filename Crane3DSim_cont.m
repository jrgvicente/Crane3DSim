%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%       J.VICENTE - j.vicente@unizar.es           LAST UPDATE: 03/07/2025
%       [V1.0]
%       Universidad de Zaragoza - Instituto de Investigacion en Ingenieria
%       de Aragon
%
%  ----------------------------------------------------------------------------
%  ----------------------------------------------------------------------------
% 
% Function to simulate the dynamics of the 3D overhead crane with continuous 
% PWM signal input
%
%
% function [T_out DATA_OUT ESTADO_OUT] = Crane3DSim_cont(frictions,IMOT,IMOTx,IMOTy,ks,uPWM_x,uPWM_y,uPWM_z,masas,x0,tsimul,g,ph_limits)
%
% ===========================
% ==== function Inputs =====
% ===========================
%
% > frictions = is a structure with the following fields:
% 
%   frictions.Tsx_positive =  @(x)  % coefficient of static friction of the trolley on axis +X (function dependent of the position)
%   frictions.Tsx_negative =  @(x)  % coefficient of static friction of the trolley on axis -X (function dependent of the position)
% 
%   frictions.Tsy_positive = @(y)  % coefficient of static friction of the trolley on axis +Y (function dependent of the position)
%   frictions.Tsy_negative = @(y)  % coefficient of static friction of the trolley on axis -Y (function dependent of the position)
% 
%   frictions.Tsr_positive =  @(l)  % coefficient of static friction of the trolley on axis +L (function dependent of the position)
%   frictions.Tsr_negative = @(l)   % coefficient of static friction of the trolley on axis -L (function dependent of the position)
% 
%   frictions.Tdy_positive = 0.0;  % coefficient of dynamic friction of the trolley on axis +Y 
%   frictions.Tdy_negative = 0.0;  % coefficient of dynamic friction of the trolley on axis -Y 
% 
%   frictions.Tdx_positive = 0.0;  % coefficient of dynamic friction of the trolley on axis +X 
%   frictions.Tdx_negative = 0.0;  % coefficient of dynamic friction of the trolley on axis -X 
% 
%   frictions.Tdr_positive = 0.0;  % coefficient of dynamic friction of the trolley on the +rope axis
%   frictions.Tdr_negative = 0.0;  % coefficient of dynamic friction of the trolley on the -rope axis
% 
%   frictions.K_AIREA = 0.0;  % coefficient of dynamic friction of the alpha angle 
%   frictions.K_AIREB = 0.0;  % coefficient of dynamic friction of the beta angle 
%
% > IMOT is the term associated with the inertia of the motor on the axis of the rope.
%
% (mc + IMOT)*dd(l) = Z_PWM_TO_F*PWM - Tdr*d(l) - Tsr*signo(d(l)) + mc*g
%
% > IMOTx and IMOTy is the term associated with the motor inertia in the X and Y axis respectively.
%
% (mtot + IMOTx)*dd(x) = X_PWM_TO_F*PEM - Tdx*d(x) - Tsx*signo(d(x)) 
%
%
% > ks = [X_PWM_TO_F Y_PWM_TO_F Z_PWM_TO_F] [N/PWM]
%
% > Input forces 
%   uPWM_x =  x-axis force input (PWM from 0 to 1) must be @(t) functions
%   uPWM_y =  y-axis force input (PWM from 0 to 1) must be @(t) functions
%   uPWM_z =  z-axis force input (PWM from 0 to 1) must be @(t) functions
%
% > masses = [mc mw ms] [Kg] [payload trolley rail]
%
% > x0 = [y0 dy0 x0 dx0 alpha0 dalpha0 beta0 dbeta0 R0 dR0] Initial conditions
%
%       y0 -- initial trolley position on y axis [m]
%       dy0 -- initial trolley velocity on y axis [m]
%       x0 -- initial trolley position on x axis [m]
%       dx0 -- initial trolley velocity on x axis [m]
%       alpha0 -- initial alpha angle [rad]
%       dalpha0 -- initial alpha anglular velocity [rad/s]
%       beta0 -- initial beta angle [rad]
%       dbeta0 -- initial beta anglular velocity [rad/s]
%       R0 -- initial cable length [m]
%       dR0 -- initial cale velocity [m/s]
%       
%
% > tsimul = simulation time in seconds
%
%
% > g = gravity [m/s2]
%
% > ph_limits = [xlim_positive xlim_negative ylim_positive ylim_negative rlim_positive rlim_negative] physical limits of overhead crane movement
%    xlim_positive -- maximum X-axis coordinate of the carriage [m]
%    xlim_negative -- minimum X-axis coordinate of the carriage [m]
%    ylim_positive -- maximum Y-axis coordinate of the carriage [m]
%    ylim_negative -- minimum Y-axis coordinate of the carriage [m]
%    rlim_positive -- maximum rope-axis coordinate of the carriage [m]
%    rlim_negative -- minimum rope-axis coordinate of the carriage [m]
%
%
% ===========================
% ==== function outputs =====
% ===========================
%
% T_out = Vector of time instants of the simulation
%
% DATA_OUT = 
% [yw dyw ddyw xw dxw ddxw alpha dalpha beta dbeta R dR ddR
% yc dyc ddyc zc dzc ddzc]
%
%   yw -- trolley y-axis position [m] vector
%   dyw -- trolley y-axis velocity [m/s] vector
%   ddyw -- trolley y-axis acceleration [m/s2] vector
%   xw -- trolley x-axis position [m] vector
%   dxw -- trolley x-axis velocity [m/s] vector
%   ddxw -- trolley x-axis acceleration [m/s2] vector
%   alpha -- alpha angle position [rad] vector
%   dalpha -- alpha angular velocity [rad/s] vector
%   beta -- beta angle position [rad] vector
%   dbeta -- beta angular velocity [rad/s] vector
%   R -- cable length [m] vector
%   dR -- cable length velocity [m/s] vector
%   ddR -- cable length acceleration [m/s2] vector
%   yc -- payload y-axis position [m] vector
%   dyc -- payload y-axis velocity [m/s] vector
%   ddyc -- payload y-axis acceleration [m/s2] vector
%   xc -- payload x-axis position [m] vector
%   dxc -- payload x-axis velocity [m/s] vector
%   ddxc -- payload x-axis acceleration [m/s2] vector
%   zc -- payload z-axis position [m] vector
%   dzc -- payload z-axis velocity [m/s] vector
%   ddzc -- payload z-axis acceleration [m/s2] vector
%
% ESTADO_OUT = hydrid model state (-1 0 1) in each axis in each time simulation instant
%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

function [T_out DATA_OUT ESTADO_OUT] = Crane3DSim_cont(frictions,IMOT,IMOTx,IMOTy,ks,uPWM_x,uPWM_y,uPWM_z,masas,x0,tsimul,g,ph_limits)

%% CTES PWM - N

X_PWM_TO_F = ks(1);
Y_PWM_TO_F = ks(2);
Z_PWM_TO_F = ks(3);

% g = 9.81;   % gravity [m/s2]


%% PHYSICAL LIMITS 

xlim_positive = ph_limits(1);
xlim_negative = ph_limits(2);
ylim_positive = ph_limits(3);
ylim_negative = ph_limits(4);
rlim_positive = ph_limits(5);
rlim_negative = ph_limits(6);  


%% FRICTION

Tsx_fun_negative = @(x) frictions.Tsx_negative(x)*X_PWM_TO_F;  
Tsx_fun_positive = @(x) frictions.Tsx_positive(x)*X_PWM_TO_F;  

Tsy_fun_positive = @(y) frictions.Tsy_positive(y)*Y_PWM_TO_F;   
Tsy_fun_negative = @(y) frictions.Tsy_negative(y)*Y_PWM_TO_F;    

Tsr_fun_positive = @(l) frictions.Tsr_positive(l)*Z_PWM_TO_F;   
Tsr_fun_negative = @(l) frictions.Tsr_negative(l)*Z_PWM_TO_F;   

Tdy_positive = frictions.Tdy_positive*Y_PWM_TO_F;  
Tdy_negative = frictions.Tdy_negative*Y_PWM_TO_F;  

Tdx_positive = frictions.Tdx_positive*X_PWM_TO_F; 
Tdx_negative = frictions.Tdx_negative*X_PWM_TO_F; 

Tdr_positive = frictions.Tdr_positive*Z_PWM_TO_F;   
Tdr_negative = frictions.Tdr_negative*Z_PWM_TO_F;  

K_AIREA = frictions.K_AIREA; 
K_AIREB = frictions.K_AIREB; 


%% MASSES

mc = masas(1);           % payload mass [Kg]
mw = masas(2);           % trolley mass [Kg] 
ms = masas(3);           % rail mass [Kg]


%% INITIAL CONDITIONS

x = x0;

% Time to do the simulation
tfinal = tsimul;      % final time [seconds]
t0 = 0;               % initial time [seconds]

t_parado =0;          % Time for the system to be stopped at startup [seconds] 
tf = tfinal+t_parado;
tspan = [t0 tf];



%% FORCE INPUTS

uF_x0 = uPWM_x;
uF_y0 = uPWM_y;
uF_r0 = uPWM_z;

uF_x = @(t) uF_x0(t)*X_PWM_TO_F;    

uF_y = @(t) uF_y0(t)*Y_PWM_TO_F;

uF_r = @(t) uF_r0(t)*Z_PWM_TO_F;

qx = 0;
qy = 0;
qr = 0;



%% OBTAIN INITIAL STATE

uF_x_aplicar = uF_x(0);
uF_y_aplicar = uF_y(0);
uF_r_aplicar = uF_r(0);

X_anterior = x;

Tdiscret = 0.002;

[qx_new, qy_new, qr_new, X_k1, T_k1] = Crane3DSim_1state_fast(Tdiscret, X_anterior, uF_x_aplicar, uF_y_aplicar , uF_r_aplicar , mw, ms, mc, Tsx_fun_positive, Tsx_fun_negative , Tsy_fun_positive, Tsy_fun_negative, Tsr_fun_positive, Tsr_fun_negative, Tdy_positive,Tdy_negative, Tdx_positive, Tdx_negative, Tdr_positive, Tdr_negative, g, K_AIREA, K_AIREB, xlim_positive, xlim_negative, ylim_positive, ylim_negative, qx, qy, qr,IMOT, IMOTx, IMOTy);
 
 
qx = qx_new;
qy = qy_new;
qr = qr_new;


if qy == 0
    x(2) = 0;
end

if qx == 0
    x(4) = 0;
end

if qr == 0
    x(10) = 0;
end



%% VVARIABLES TO STORE RESULTS

T_total = [];
X_total = [];

registro_IE = [];
registro_q = [];

%% SOLVER - ODE

while tspan(1) < tf
    aux_registro_q = [tspan(1) qx qy qr];
    registro_q = [registro_q; aux_registro_q];

    % EVENTS FUNCTION:
    funcion_eventos = @(t, x) EventsFnc(t, x, uF_x, uF_y, uF_r, qx, qy, qr, Tsx_fun_positive, Tsx_fun_negative , Tsy_fun_positive, Tsy_fun_negative, Tsr_fun_positive, Tsr_fun_negative, Tdy_positive,Tdy_negative, Tdx_positive, Tdx_negative, Tdr_positive, Tdr_negative , g, mc,mw,ms, xlim_positive, xlim_negative, ylim_positive,ylim_negative,rlim_positive,rlim_negative) ; 

    % DYNAMIC FUNCTION:
    funcion_ode = @(t, x) func_modo1(t, x, uF_x, uF_y, uF_r, mw, ms, mc, Tsx_fun_positive, Tsx_fun_negative , Tsy_fun_positive, Tsy_fun_negative, Tsr_fun_positive, Tsr_fun_negative, Tdy_positive,Tdy_negative, Tdx_positive, Tdx_negative, Tdr_positive, Tdr_negative, g, K_AIREA, K_AIREB, qx, qy, qr);

    % Solving the differential equation
    options = odeset('Events', funcion_eventos, 'AbsTol', 1e-13, 'RelTol',1e-13);% 'MaxStep',1e-4);
   
    
    [T, X_new, TE, YE, IE] = ode45(funcion_ode, tspan, x, options);
   
        
    % Saving the results
    T_total = [T_total; T];
    X_total = [X_total; X_new];
    
    % Updating the time
    tspan(1) = T(end);

    % New initial state
    x = X_new(end, :);


    % Calculate the force being done by the rope

            uF_x_apl = uF_x(tspan(1));
            uF_y_apl = uF_y(tspan(1));
            uF_r_apl = uF_r(tspan(1));
  
       
        if qr == 0                                                              % Does not move on the axis of the rope
            S = - g*mc*(cos(x(7))*sin(x(5)));        
        elseif qr == 1                                                          % Moves upward x(10)<0
            Tr = Tdr_positive*x(10) - Tsr_fun_positive(x(9));                             
            S = uF_r_apl - Tr;
        elseif qr == -1                                                         % Moves downward x(10)>0    
            Tr = Tdr_negative*x(10) + Tsr_fun_negative(x(9));                             
            S = uF_r_apl - Tr;
        end 
       
   
    
    % Change to the next state checking if there has been any event
        if ~isempty(IE)                                                     
    
                % x
    
                if (  any(IE == 1) && qx == 0)                                  % force towards +x exceeds dry friction and was stationary
                    qx = 1;                                                     % Start at x to (+x)
        
                elseif (  any(IE == 2) && qx == 0)                              % the force towards -x exceeds the dry friction and was stationary
                    qx = -1;                                                    % Start at x to (-x)
        
                elseif (  any(IE == 3) ||  any(IE == 10)  || any(IE == 11))                                                % if it was moving and it went to speed 0
                    if (((uF_x_apl - sin(x(5))*sin(x(7))*S) > Tsx_fun_positive(x(3))) && (x(3)<xlim_positive))             % dont stand still because I move from -x to +x.
                        qx = 1;
                    elseif (((uF_x_apl - sin(x(5))*sin(x(7))*S) < -Tsx_fun_negative(x(3))) && (x(3)>xlim_negative))        % dont stand still because I move from +x to -x.
                        qx = -1;
                    else
                        qx = 0;                                                 % changeover to stop status
                        x(4) = 0;
                    end                
                end
    
                
                % y
    
                if (  any(IE == 4) && qy == 0)                                  % force towards +y exceeds dry friction and was stationary
                    qy = 1;                                                     % Start at y to (+y)
        
                elseif (  any(IE == 5) && qy == 0)                              % the force towards - and overcomes dry friction and was stationary
                    qy = -1;                                                    % Starts at and to (-y)
        
                elseif (  any(IE == 6)  ||  any(IE == 12)  || any(IE == 13))                                     % if it was moving and it went to speed 0
                    if (((uF_y_apl -S*cos(x(5))) > Tsy_fun_positive(x(1))) && (x(1)<ylim_positive))              % dont stand still because  move from -y to +y.
                        qy = 1;
                    elseif (((uF_y_apl -S*cos(x(5))) < -Tsy_fun_negative(x(1))) && (x(1)>ylim_negative))         % dont stand still because  move from +y to -y.
                        qy = -1;
                    else
                        qy = 0;                                                 % changeover to stop status
                        x(2) = 0;
                    end    
    
                end
    
    
    
                % cuerda
    
                if (  any(IE == 7) && qr == 0)                                  % the upward force overcomes the dry friction and was stationary
                    qr = 1;                                                     % Starts upward
    
                elseif (  any(IE == 8) && qr == 0)                              % downward force overcomes dry friction and was stationary
                    qr = -1;                                                    % Starts downward
    
                elseif (  any(IE == 9) ||  any(IE == 14)  || any(IE == 15))     % if I was moving and I went to speed 0
                    
                    
                     % -- x --
    
                    if qx == 1                  
                        dx(4) =  (uF_x_apl -(Tdx_positive*x(4) + Tsx_fun_positive(x(3))) - S*sin(x(5))*sin(x(7)))/(ms + mw + IMOTx);                    
                    elseif qx == -1             
                        dx(4) =  (uF_x_apl -(Tdx_negative*x(4) - Tsx_fun_negative(x(3))) - S*sin(x(5))*sin(x(7)))/(ms + mw + IMOTx);                 
                    else
                        dx(4) = 0;                    
                    end
    
    
                    % -- y --
    
                    if qy == 1                  
                        dx(2) = (uF_y_apl - (Tdy_positive*x(2) + Tsy_fun_positive(x(1))) - S*cos(x(5)))/(mw + IMOTy);                    
                    elseif qx == -1             
                        dx(2) = (uF_y_apl - (Tdy_negative*x(2) - Tsy_fun_negative(x(1))) - S*cos(x(5)))/(mw + IMOTy);             
                         
                    else
                        dx(2) = 0;
                        
                    end
    
    
                    % -- rope --
    
                    S1 = uF_r_apl;
    
    
                    f_cuerda = (S1 - mc*cos(x(5))*dx(2) + mc*x(9)*x(6)^2 + mc*x(9)*x(8)^2 - mc*cos(x(5))^2*x(9)*x(8)^2 - mc*sin(x(5))*sin(x(7))*dx(4) + g*mc*cos(x(7))*sin(x(5)) );
    
                   
                    if (f_cuerda < -Tsr_fun_positive(x(9))) && (x(9)>rlim_negative)                             
                        qr = 1;
                    elseif (f_cuerda > Tsr_fun_negative(x(9))) && (x(9)<rlim_positive)                           
                        qr = -1;
                    else
                        qr = 0;                                                 
                        x(10) = 0;
                    end
                end


        end


    

    % Eliminate repeated points
    [T_total, idx] = unique(T_total);
    X_total = X_total(idx,:);
end


% I save the last state in which it is in each axis.

aux_registro_q = [T_total(end) qx qy qr];
registro_q = [registro_q; aux_registro_q];



%% DYNAMIC FUNCTION

function dx_dt = func_modo1(t, x, uF_x_aplicar2, uF_y_aplicar2, uF_r_aplicar2, mw, ms, mc, Tsx_fun_positive, Tsx_fun_negative , Tsy_fun_positive, Tsy_fun_negative, Tsr_fun_positive, Tsr_fun_negative, Tdy_positive,Tdy_negative, Tdx_positive, Tdx_negative, Tdr_positive, Tdr_negative, g, K_AIREA, K_AIREB, qx, qy, qr)
 

    uF_x_aplicar = uF_x_aplicar2(t);
    uF_y_aplicar = uF_y_aplicar2(t);
    uF_r_aplicar = uF_r_aplicar2(t);


%% ROPE TENSION
    if qr == 0                                                              % No movement along the rope's axis
        S =  -g*mc*(cos(x(7))*sin(x(5)));
    elseif qr == 1                                                          % Rope moves upwards                                                         
        Tr = Tdr_positive*x(10) - Tsr_fun_positive(x(9));                             % upwards, x(10)<0 and the total force must be <0    
        S = uF_r_aplicar-Tr;       
    elseif qr == -1                                                         % It moves downwards (negative velocity)        
        Tr = Tdr_negative*x(10) + Tsr_fun_negative(x(9));                             % downwards, x(10)>0 and the total force must be >0    
        S = uF_r_aplicar-Tr;       
    end     

%% X-AXIS
    if qx == 0                                                              % No movement in x
        Tx = uF_x_aplicar;
        dx(3) = 0;                                                          % dxw    
        dx(4) = 0;                                                          % ddxw        
    elseif qx == 1                                                          % It moves in the +x direction (positive velocity)
        Tx = Tdx_positive*x(4) + Tsx_fun_positive(x(3));                              % [N] (absolute value, with direction but no sign)
        dx(3) = x(4);                                                       % dxw            
        dx(4) =  (uF_x_aplicar -Tx - S*sin(x(5))*sin(x(7)))/(ms + mw + IMOTx);           % ddxw        
    elseif qx == -1                                                         % It moves in the -x direction (negative velocity)
        Tx = Tdx_negative*x(4) - Tsx_fun_negative(x(3));                              % [N] (absolute value, with direction but no sign)
        dx(3) = x(4);                                                       % dxw         
        dx(4) =  (uF_x_aplicar -Tx - S*sin(x(5))*sin(x(7)))/(ms + mw + IMOTx);           % ddxw        
    end

%% Y-AXIS
    if qy == 0                                                              % No movement in y
        Ty = uF_y_aplicar;
        dx(1) = 0;                                                          % dyw    
        dx(2) = 0;                                                          % ddyw        
    elseif qy == 1                                                          % It moves in the +y direction (positive velocity)
        Ty = Tdy_positive*x(2) + Tsy_fun_positive(x(1));                              % [N] (absolute value, with direction but no sign)
        dx(1) = x(2);                                                       % dyw
        dx(2) = (uF_y_aplicar - Ty - S*cos(x(5)))/(mw + IMOTy);                            % ddyw        
    elseif qy == -1                                                         % It moves in the -y direction (negative velocity)
        Ty = Tdy_negative*x(2) -Tsy_fun_negative(x(1));                               % [N] (absolute value, with direction but no sign)
        dx(1) = x(2);                                                       % dyw
        dx(2) = (uF_y_aplicar -Ty - S*cos(x(5)))/(mw + IMOTy);                             % ddyw        
    end

%% ROPE MOVEMENT
    if qr == 0                                                              % No movement along the rope's axis        
        dx(9) = 0;                                                          % dR        
        dx(10) = 0;                                                         % ddR
    elseif qr == 1                                                          % It moves upwards       
        dx(9) = x(10);         
        dx(10) = (1/(mc+IMOT)) * (S - mc*cos(x(5))*dx(2) + mc*x(9)*x(6)^2 + mc*x(9)*x(8)^2 - mc*cos(x(5))^2*x(9)*x(8)^2 - mc*sin(x(5))*sin(x(7))*dx(4) + g*mc*cos(x(7))*sin(x(5)) );
    elseif qr == -1                                                         % It moves downwards (negative velocity)      
        dx(9) = x(10);                                                      % dR   
        dx(10) = (1/(mc+IMOT)) * (S - mc*cos(x(5))*dx(2) + mc*x(9)*x(6)^2 + mc*x(9)*x(8)^2 - mc*cos(x(5))^2*x(9)*x(8)^2 - mc*sin(x(5))*sin(x(7))*dx(4) + g*mc*cos(x(7))*sin(x(5)) );
    end



dx(5) = x(6);
dx(7) = x(8);

dx(6) = (1/x(9)) * (-2*x(10)*x(6) + sin(x(5))*dx(2) - cos(x(5))*sin(x(7))*dx(4) + g*cos(x(5))*cos(x(7)) + cos(x(5))*sin(x(5))*x(9)*x(8)^2  ) - 1/(mc*x(9)^2)*K_AIREA*x(6);     % ddALFA

dx(8) = (1/ (sin(x(5))*x(9))) * (-g*sin(x(7)) -cos(x(7))*dx(4) -2*sin(x(5))*x(10)*x(8) - 2*cos(x(5))*x(9)*x(6)*x(8) )  - 1/(mc*x(9)^2)*K_AIREB*x(8); % ddBETA


dx_dt = transpose(dx);

end


%% EVENTS FUNCTION

function [value, isterminal, direction] = EventsFnc(t, x, uF_x_aplicar2, uF_y_aplicar2, uF_r_aplicar2, qx, qy, qr, Tsx_fun_positive, Tsx_fun_negative , Tsy_fun_positive, Tsy_fun_negative, Tsr_fun_positive, Tsr_fun_negative, Tdy_positive,Tdy_negative, Tdx_positive, Tdx_negative, Tdr_positive, Tdr_negative , g, mc,mw,ms, xlim_positive, xlim_negative, ylim_positive,ylim_negative,rlim_positive,rlim_negative)
    
    
    uF_x_aplicar = uF_x_aplicar2(t);
    uF_y_aplicar = uF_y_aplicar2(t);
    uF_r_aplicar = uF_r_aplicar2(t);
    
    
    % Calculate the force exerted by the rope
    % S = uF_r(t);
    
    if qr == 0                                                              % No movement along the rope's axis
        S = - g*mc*(cos(x(7))*sin(x(5)));
    elseif qr == 1                                                          % It moves upwards, x(10)<0
        % [N] (absolute value, with direction but no sign) For the rope, when it extends, r_dot is positive,
        % but the positive force is defined to shorten the rope.
        Tr = Tdr_positive*x(10) - Tsr_fun_positive(x(9));
        S = uF_r_aplicar - Tr;
    elseif qr == -1                                                         % It moves downwards, x(10)>0
        % [N] (absolute value, with direction but no sign) For the rope, when it extends, r_dot is positive,
        % but the positive force is defined to shorten the rope.
        Tr = Tdr_negative*x(10) + Tsr_fun_negative(x(9));
        S = uF_r_aplicar - Tr;
    end
    
    % Here I want to detect that the forces are greater than the
    % dry friction for the movement to start.

    %% X-AXIS
    
    % MOVEMENT TOWARDS +X (uF_x(t) will be positive); when it changes from - to +,
    % it starts moving towards +X (here, only the - to + change is detected)
    f1x = ((uF_x_aplicar - sin(x(5))*sin(x(7))*S) - Tsx_fun_positive(x(3))) * (x(3)<xlim_positive);
    % disp(f1x)
    
    % MOVEMENT TOWARDS -X (uF_x(t) will be negative); when it changes from + to -,
    % it starts moving towards -X (here, only the + to - change is detected)
    f2x = ((uF_x_aplicar - sin(x(5))*sin(x(7))*S) + Tsx_fun_negative(x(3))) * (x(3)>xlim_negative);
    
    % DETECT WHEN THE VELOCITY IN X IS 0 (this is detected for both
    % directions, considering the current state)
    f3x = (x(4));
    if qx == 1          % it moves towards +x (x(4)>0), I want to detect the change from + to -
        d3x = -1;
    elseif qx == -1     % it moves towards -x (x(4)<0), I want to detect the change from - to +
        d3x = +1;
    else
        d3x = 5;        % I put a nonsense value as it's not used in this case, to cause an error if it is
    end
    
    % DETECT THE POSITION TO STOP IF X >= MAXIMUM X LIMIT
    f4x = xlim_positive-x(3);
    d4x = -1;
    
    % DETECT THE POSITION TO STOP IF X <= MINIMUM X LIMIT
    f5x = xlim_negative-x(3);
    d5x = 1;
    
    %% Y-AXIS
    
    % MOVEMENT TOWARDS +Y (uF_y(t) will be positive); when it changes from - to +,
    % it starts moving towards +Y (here, only the - to + change is detected)
    f1y = ((uF_y_aplicar -S*cos(x(5))) - Tsy_fun_positive(x(1))) * (x(1)<ylim_positive);
    
    % MOVEMENT TOWARDS -Y (uF_y(t) will be negative); when it changes from + to -,
    % it starts moving towards -Y (here, only the + to - change is detected)
    f2y = ((uF_y_aplicar -S*cos(x(5))) + Tsy_fun_negative(x(1))) * (x(1)>ylim_negative);
    
    % DETECT WHEN THE VELOCITY IN Y IS 0 (this is detected for both
    % directions, considering the current state)
    f3y = (x(2));
    if qy == 1          % it moves towards +y (x(2)>0), I want to detect the change from + to -
        d3y = -1;
    elseif qy == -1     % it moves towards -y (x(2)<0), I want to detect the change from - to +
        d3y = +1;
    else
        d3y = 5;        % I put a nonsense value as it's not used in this case, to cause an error if it is
    end
    
    % DETECT THE POSITION TO STOP IF Y >= MAXIMUM Y LIMIT
    f4y = ylim_positive-x(1);
    d4y = -1;
    
    % DETECT THE POSITION TO STOP IF Y <= MINIMUM Y LIMIT
    f5y = ylim_negative-x(1);
    d5y = 1;
    
    %% ROPE AXIS
    
    if qx == 1                  % it moves in +X
        dx(4) =  (uF_x_aplicar - (Tdx_positive*x(4) + Tsx_fun_positive(x(3))) - S*sin(x(5))*sin(x(7)))/(ms + mw + IMOTx);
    elseif qx == -1             % it moves in -X
        dx(4) =  (uF_x_aplicar -(Tdx_negative*x(4) - Tsx_fun_negative(x(3))) - S*sin(x(5))*sin(x(7)))/(ms + mw + IMOTx);
    else
        dx(4) = 0;
    end
    % -- y --
    if qy == 1                  % it moves in +Y
        dx(2) = (uF_y_aplicar - (Tdy_positive*x(2) + Tsy_fun_positive(x(1))) - S*cos(x(5)))/(mw + IMOTy);
    elseif qy == -1             % it moves in -Y
        dx(2) = (uF_y_aplicar - (Tdy_negative*x(2) - Tsy_fun_negative(x(1))) - S*cos(x(5)))/(mw + IMOTy);
    else
        dx(2) = 0;
    end
    % -- rope --
    S1 = uF_r_aplicar;
    f_cuerda = (S1 - mc*cos(x(5))*dx(2) + mc*x(9)*x(6)^2 + mc*x(9)*x(8)^2 - mc*cos(x(5))^2*x(9)*x(8)^2 - mc*sin(x(5))*sin(x(7))*dx(4) + g*mc*cos(x(7))*sin(x(5)) );
    
    % MOVEMENT UPWARDS (ftot_rope will be negative); when it changes from + to -,
    % it starts moving upwards (here, only the + to - change is detected)
    f1r = f_cuerda + Tsr_fun_positive(x(9));
    %f1r = ftot_cuerda_hacia_arriba;
    
    % MOVEMENT DOWNWARDS (ftot_rope will be positive); when it changes from - to +,
    % it starts moving downwards (here, only the - to + change is detected)
    f2r = f_cuerda - Tsr_fun_negative(x(9));
    % f2r = ftot_cuerda_hacia_abajo;
    
    % DETECT WHEN THE VELOCITY IN R IS 0 (this is detected for both
    % directions, considering the current state)
    f3r = (x(10));
    if qr == 1          % it moves upwards (x(10)<0), I want to detect the change from - to +
        d3r = +1;
    elseif qr == -1     % it moves downwards (x(10)>0), I want to detect the change from + to -
        d3r = -1;
    else
        d3r = 5;        % I put a nonsense value as it's not used in this case, to cause an error if it is
    end
    
    % DETECT THE POSITION TO STOP IF ROPE LENGTH >= MAXIMUM ROPE LENGTH LIMIT
    f4r = rlim_positive-x(9);
    d4r = -1;
    
    % DETECT THE POSITION TO STOP IF ROPE LENGTH <= MINIMUM ROPE LENGTH LIMIT
    f5r = rlim_negative-x(9);
    d5r = 1;        
    
    value = [f1x; f2x; f3x; f1y; f2y; f3y; f1r; f2r; f3r; f4x; f5x; f4y; f5y;f4r;f5r];        
    isterminal = [1;1;1;1;1;1;1;1;1;1;1;1;1;1;1];                                   
    direction = [1;-1;d3x;1;-1;d3y;-1;+1;d3r;d4x;d5x;d4y;d5y;d4r;d5r];                      

end



%::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::
%                    GRAPHS AND RESULTS
%::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::



%% EXPAND STATES TO A VECTOR WITH THE SAME SIZE AS T_total


t_eventos = registro_q(:,1);  
datos_eventos = registro_q(:,2:end);  
registro_q_expandido = zeros(length(T_total), size(datos_eventos,2));

for i = 1:size(datos_eventos,2)
    registro_q_expandido(:,i) = interp1(t_eventos, datos_eventos(:,i), T_total, 'previous', 'extrap');
end


%% RELATIVE AND ABSOLUTE POSITION OF THE LOAD

x_rel_carga = X_total(:,9).*sin(X_total(:,5)).*sin(X_total(:,7));
y_rel_carga = X_total(:,9).*cos(X_total(:,5));
z_rel_carga = - X_total(:,9).*sin(X_total(:,5)).*cos(X_total(:,7));

pos_rel_carga = [x_rel_carga y_rel_carga z_rel_carga];

x_abs_carga = x_rel_carga + X_total(:,3);
y_abs_carga = y_rel_carga + X_total(:,1);

pos_abs_carga = [x_abs_carga y_abs_carga z_rel_carga];


%%%%%
%% A posteriori calculation of trolley and load positions and accelerations
%%%%%


dxc = X_total(:,4) + sin(X_total(:,5)).*sin(X_total(:,7)).*X_total(:,10) + cos(X_total(:,5)).*sin(X_total(:,7)).*X_total(:,9).*X_total(:,6) + cos(X_total(:,7)).*sin(X_total(:,5)).*X_total(:,9).*X_total(:,8);
dyc = cos(X_total(:,5)).*X_total(:,10) + X_total(:,2) - sin(X_total(:,5)).*X_total(:,9).*X_total(:,6);
dzc = sin(X_total(:,5)).*sin(X_total(:,7)).*X_total(:,9).*X_total(:,8) - cos(X_total(:,5)).*cos(X_total(:,7)).*X_total(:,9).*X_total(:,6) - cos(X_total(:,7)).*sin(X_total(:,5)).*X_total(:,10);

ddxc = zeros(size(dxc));
ddyc = zeros(size(dyc));
ddzc = zeros(size(dzc));

ddR_vec = zeros(size(dzc));

ddyw_vec = zeros(size(dxc));
ddxw_vec = zeros(size(dxc));

UFR_vec = zeros(size(dzc));
UFY_vec = zeros(size(dzc));
UFX_vec = zeros(size(dzc));




for i = 1:size(T_total,1)
    x = X_total(i,:);
    t = T_total(i);
    qx = registro_q_expandido(i,1);
    qy = registro_q_expandido(i,2);
    qr = registro_q_expandido(i,3);
     
           
    if qr == 0                                                             
        S = - g*mc*(cos(x(7))*sin(x(5)));        
    elseif qr == 1                                                          
        Tr = Tdr_positive*x(10) - Tsr_fun_positive(x(9));                   
        S = uF_r(t) - Tr;
    elseif qr == -1                                                         
        Tr = Tdr_negative*x(10) + Tsr_fun_negative(x(9));                   
        S = uF_r(t) - Tr;
    end 


    %% X-AXIS
    
    if qx == 0                                                              
        ddxw = 0;
    elseif qx == 1                                                          
        Tx = Tdx_positive*x(4) + Tsx_fun_positive(x(3));                             
        ddxw =  (uF_x(t) -Tx - S*sin(x(5))*sin(x(7)))/(ms + mw + IMOTx);             
    elseif qx == -1                                                        
        Tx = Tdx_negative*x(4) - Tsx_fun_negative(x(3));                                  
        ddxw =  (uF_x(t) -Tx - S*sin(x(5))*sin(x(7)))/(ms + mw + IMOTx);           
    end


    %% Y-AXIS

    if qy == 0                                                              
        ddyw = 0;
    elseif qy == 1                                                          
        Ty = Tdy_positive*x(2) + Tsy_fun_positive(x(1));                             
        ddyw = (uF_y(t) - Ty - S*cos(x(5)))/(mw + IMOTy);                                
    elseif qy == -1                                                        
        Ty = Tdy_negative*x(2) -Tsy_fun_negative(x(1));                               
        ddyw = (uF_y(t) -Ty - S*cos(x(5)))/(mw + IMOTy);                             
    end


    %% ROPE

    if qr == 0                                                                   
        ddR = 0;                                                            
    elseif qr == 1                                                              
        ddR = (1/(mc+IMOT)) * (S - mc*cos(x(5))*ddyw + mc*x(9)*x(6)^2 + mc*x(9)*x(8)^2 - mc*cos(x(5))^2*x(9)*x(8)^2 - mc*sin(x(5))*sin(x(7))*ddxw + g*mc*cos(x(7))*sin(x(5)) );
    elseif qr == -1                                                             
        ddR = (1/(mc+IMOT)) * (S - mc*cos(x(5))*ddyw + mc*x(9)*x(6)^2 + mc*x(9)*x(8)^2 - mc*cos(x(5))^2*x(9)*x(8)^2 - mc*sin(x(5))*sin(x(7))*ddxw + g*mc*cos(x(7))*sin(x(5)) );
    end   
    
    
    
    dda = (1/x(9)) * (-2*x(10)*x(6) + sin(x(5))*ddyw - cos(x(5))*sin(x(7))*ddxw + g*cos(x(5))*cos(x(7)) + cos(x(5))*sin(x(5))*x(9)*x(8)^2  ) - K_AIREA*x(6);     

    ddb = (1/ (sin(x(5))*x(9))) * (-g*sin(x(7)) -cos(x(7))*ddxw -2*sin(x(5))*x(10)*x(8) - 2*cos(x(5))*x(9)*x(6)*x(8) )  - K_AIREB*x(8); 
    
     
 
    ddyw_vec(i) = ddyw;
    ddxw_vec(i) = ddxw;
    ddR_vec(i) = ddR;
    
    
   
    ddxc(i) = sin(x(5))*sin(x(7))*ddR + ddxw + 2*cos(x(5))*sin(x(7))*x(10)*x(6) + 2*cos(x(7))*sin(x(5))*x(10)*x(8) - sin(x(5))*sin(x(7))*x(9)*x(6)^2 - sin(x(5))*sin(x(7))*x(9)*x(8)^2 + cos(x(5))*sin(x(7))*x(9)*dda + cos(x(7))*sin(x(5))*x(9)*ddb + 2*cos(x(5))*cos(x(7))*x(9)*x(6)*x(8);
    ddyc(i) = cos(x(5))*ddR + ddyw - cos(x(5))*x(9)*x(6)^2 - sin(x(5))*x(9)*dda - 2*sin(x(5))*x(10)*x(6);
    ddzc(i) = 2*sin(x(5))*sin(x(7))*x(10)*x(8) - 2*cos(x(5))*cos(x(7))*x(10)*x(6) - cos(x(7))*sin(x(5))*ddR + cos(x(7))*sin(x(5))*x(9)*x(6)^2 + cos(x(7))*sin(x(5))*x(9)*x(8)^2 - cos(x(5))*cos(x(7))*x(9)*dda + sin(x(5))*sin(x(7))*x(9)*ddb + 2*cos(x(5))*sin(x(7))*x(9)*x(6)*x(8);

    UFR_vec(i) = -uF_r(t);
    UFY_vec(i) = uF_y(t);
    UFX_vec(i) = uF_x(t);

end





%%%%%%%%%%%%%%%%%%%%%
%% ..:: Force graphs: ::..
%%%%%%%%%%%%%%%%%%%%%

Tsy_aplicada_vec = zeros(size(T_total,1),1);
Tr_aplicada_vec = zeros(size(T_total,1),1);

% First I get the real applied f taking into account friction.

% :: X ::

fx_ext_con_roz_vec = zeros(size(T_total));

for i = 1:length(T_total)
    tiempo_actual = T_total(i);

    estado = registro_q_expandido(i,1);

    if estado == 0
        fx_ext_con_roz_vec(i) = 0; 

    elseif estado == 1  

        Tx = Tdx_positive*X_total(i,4) + Tsx_fun_positive(X_total(i,3)); 
        fx_ext_con_roz_vec(i) = uF_x(tiempo_actual) -Tx;


    elseif estado == -1  

        Tx = Tdx_negative*X_total(i,4) - Tsx_fun_negative(X_total(i,3));
        fx_ext_con_roz_vec(i) = uF_x(tiempo_actual) -Tx;      

    end

end


% :: Y ::

fy_ext_con_roz_vec = zeros(size(T_total));


for i = 1:length(T_total)    
    tiempo_actual = T_total(i);

    estado = registro_q_expandido(i,2);

    if estado == 0
        fy_ext_con_roz_vec(i) = 0;
        Ty = 0;

    elseif estado == 1  

        Ty = Tdy_positive*X_total(i,2) + Tsy_fun_positive(X_total(i,1));
        fy_ext_con_roz_vec(i) = uF_y(tiempo_actual) -Ty;    

    elseif estado == -1  

        Ty = Tdy_negative*X_total(i,2) - Tsy_fun_negative(X_total(i,1));
        fy_ext_con_roz_vec(i) = uF_y(tiempo_actual) -Ty;      

    end


end


% :: rope ::

fuerza_r_externa = arrayfun(uF_r, T_total);

fuerza_r_masa = g.*mc.*(cos(X_total(:,7)).*sin(X_total(:,5)));

F_tot_eje_cuerda_sin_rozamiento = zeros(size(T_total));
F_tot_eje_cuerda = zeros(size(T_total));

S_vec = zeros(size(T_total));

SumF_ejeL_masa = zeros(size(T_total));  % Sum of forces on the rope axis on the mass (if the sum is 0, the mass does not move on the rope axis, the rope does not change length).

fuerza_r_sin_rozamientos = zeros(size(T_total));


for i = 1:length(T_total)
    estado = registro_q_expandido(i,3);
    tiempo_actual = T_total(i);

    if estado == 0
        S_vec(i) = - g*mc*(cos(X_total(i,7))*sin(X_total(i,5)));   


        F_tot_eje_cuerda_sin_rozamiento(i) = (S_vec(i) - mc*cos(X_total(i,5))*ddyw_vec(i) + mc*X_total(i,9)*X_total(i,6)^2 + mc*X_total(i,9)*X_total(i,8)^2 - mc*cos(X_total(i,5))^2*X_total(i,9)*X_total(i,8)^2 - mc*sin(X_total(i,5))*sin(X_total(i,7))*ddxw_vec(i) + g*mc*cos(X_total(i,7))*sin(X_total(i,5)) );


        F_tot_eje_cuerda(i) = 0;

        Tr = 0;


    elseif estado == 1 

        Tr = Tdr_positive*X_total(i,10) - Tsr_fun_positive(X_total(i,9));
        S_vec(i) = uF_r(tiempo_actual) - Tr;        

        F_tot_eje_cuerda(i) = (S_vec(i) - mc*cos(X_total(i,5))*ddyw_vec(i) + mc*X_total(i,9)*X_total(i,6)^2 + mc*X_total(i,9)*X_total(i,8)^2 - mc*cos(X_total(i,5))^2*X_total(i,9)*X_total(i,8)^2 - mc*sin(X_total(i,5))*sin(X_total(i,7))*ddxw_vec(i) + g*mc*cos(X_total(i,7))*sin(X_total(i,5)) );

        F_tot_eje_cuerda_sin_rozamiento(i) = (uF_r(tiempo_actual) - mc*cos(X_total(i,5))*ddyw_vec(i) + mc*X_total(i,9)*X_total(i,6)^2 + mc*X_total(i,9)*X_total(i,8)^2 - mc*cos(X_total(i,5))^2*X_total(i,9)*X_total(i,8)^2 - mc*sin(X_total(i,5))*sin(X_total(i,7))*ddxw_vec(i) + g*mc*cos(X_total(i,7))*sin(X_total(i,5)) );

      
    elseif estado == -1 

        Tr = Tdr_negative*X_total(i,10) + Tsr_fun_negative(X_total(i,9));
        S_vec(i) = uF_r(tiempo_actual) - Tr;

      
        F_tot_eje_cuerda(i) = (S_vec(i) - mc*cos(X_total(i,5))*ddyw_vec(i) + mc*X_total(i,9)*X_total(i,6)^2 + mc*X_total(i,9)*X_total(i,8)^2 - mc*cos(X_total(i,5))^2*X_total(i,9)*X_total(i,8)^2 - mc*sin(X_total(i,5))*sin(X_total(i,7))*ddxw_vec(i) + g*mc*cos(X_total(i,7))*sin(X_total(i,5)) );

        F_tot_eje_cuerda_sin_rozamiento(i) = (uF_r(tiempo_actual) - mc*cos(X_total(i,5))*ddyw_vec(i) + mc*X_total(i,9)*X_total(i,6)^2 + mc*X_total(i,9)*X_total(i,8)^2 - mc*cos(X_total(i,5))^2*X_total(i,9)*X_total(i,8)^2 - mc*sin(X_total(i,5))*sin(X_total(i,7))*ddxw_vec(i) + g*mc*cos(X_total(i,7))*sin(X_total(i,5)) );

        end

    Tr_aplicada_vec(i) = Tr;
end


% ..:: x ::..

fuerza_x_externa = arrayfun(uF_x, T_total);

fuerza_x_sin_rozamientos = fuerza_x_externa - sin(X_total(:,5)).*sin(X_total(:,7)).*S_vec;

fuerza_x_final = zeros(size(T_total));


for i = 1:length(T_total)
    
    estado = registro_q_expandido(i,1);

    
    if estado == 0
        fuerza_x_final(i) = 0;
    elseif estado == 1
        Tx = Tdx_positive*X_total(i,4) + Tsx_fun_positive(X_total(i,3));
        fuerza_x_final(i) = fuerza_x_sin_rozamientos(i) - Tx;
    elseif estado == -1
        Tx = Tdx_negative*X_total(i,4) - Tsx_fun_negative(X_total(i,3));
        fuerza_x_final(i) = fuerza_x_sin_rozamientos(i) - Tx;
    end
end

% ..:: y ::..

fuerza_y_externa = arrayfun(uF_y, T_total);

fuerza_y_sin_rozamientos = fuerza_y_externa - cos(X_total(:,5)).*S_vec;

fuerza_y_final = zeros(size(T_total));


for i = 1:length(T_total)
    
    estado = registro_q_expandido(i,2);

    
    if estado == 0
        fuerza_y_final(i) = 0;
        Ty = 0;
    elseif estado == 1
        Ty = Tdy_positive*X_total(i,2) + Tsy_fun_positive(X_total(i,1));
        fuerza_y_final(i) = fuerza_y_sin_rozamientos(i) - Ty;
    elseif estado == -1
        Ty = Tdy_negative*X_total(i,2) - Tsy_fun_negative(X_total(i,1));
        fuerza_y_final(i) = fuerza_y_sin_rozamientos(i) - Ty;
    end

    Tsy_aplicada_vec(i) = Ty;
end



% ::::::: Graph of applied forces  ::::::::::::::::::::::::::::::


% Colors for the different states of the system
colores = [1, 1, 0;   % State 0 
           1, 0, 0;   % State -1 
           0, 1, 0];  % State 1 




figure(500);

%%   Subplot para la fuerza en el eje X
subplot(3, 1, 1);

% Iterate over the records to draw the patches
for i = 1:size(registro_q, 1)
    
    x_inicio = registro_q(i, 1);

    if i < size(registro_q, 1)
        x_fin = registro_q(i + 1, 1);
    else
        x_fin = T_total(end);  
    end

    estado = registro_q(i, 2);

    if estado == 0
        color_fondo = colores(1, :);
    elseif estado == -1
        color_fondo = colores(2, :);
    elseif estado == 1
        color_fondo = colores(3, :);
    end

        patch([x_inicio, x_fin, x_fin, x_inicio], ...
          [-5, -5, 5, 5], ...
          color_fondo, 'EdgeColor', 'none', 'FaceAlpha', 0.3);
end

hold on

plot(T_total, fuerza_x_externa, 'b', 'LineWidth', 1.5, 'DisplayName', 'External F' );
plot(T_total, fuerza_x_sin_rozamientos, 'r:', 'LineWidth', 1.5, 'DisplayName', 'F without friction' );
plot(T_total, fuerza_x_final, 'g:', 'LineWidth', 1.5, 'DisplayName', 'F final' );
title('Forces X-axis');
ylabel('Froce (N)');
xlabel('Time (s)');


h_legends = findobj(gca, '-regexp', 'DisplayName', '.+');
legend(h_legends);

grid on;


%% Subplot for force on Y-axis
subplot(3, 1, 2);


for i = 1:size(registro_q, 1)
    
    x_inicio = registro_q(i, 1);

    
    if i < size(registro_q, 1)
        x_fin = registro_q(i + 1, 1);
    else
        x_fin = T_total(end);  
    end
    
    estado = registro_q(i, 3);
    
    if estado == 0
        color_fondo = colores(1, :);
    elseif estado == -1
        color_fondo = colores(2, :);
    elseif estado == 1
        color_fondo = colores(3, :);
    end

        patch([x_inicio, x_fin, x_fin, x_inicio], ...
          [-5, -5, 5, 5], ...
          color_fondo, 'EdgeColor', 'none', 'FaceAlpha', 0.3);
end

hold on

plot(T_total, fuerza_y_externa, 'b', 'LineWidth', 1.5, 'DisplayName', 'External F' );
plot(T_total, fuerza_y_sin_rozamientos, 'r:', 'LineWidth', 1.5, 'DisplayName', 'F without friction' );
plot(T_total, fuerza_y_final, 'g:', 'LineWidth', 1.5, 'DisplayName', 'F final' );
title('Forces Y-axis');
ylabel('Froce (N)');
xlabel('Time (s)');

h_legends = findobj(gca, '-regexp', 'DisplayName', '.+');
legend(h_legends);

grid on;



%% Subplot for the force on the rope axis
subplot(3, 1, 3);

for i = 1:size(registro_q, 1)    
    x_inicio = registro_q(i, 1);

    
    if i < size(registro_q, 1)
        x_fin = registro_q(i + 1, 1);
    else
        x_fin = T_total(end);  
    end

    
    estado = registro_q(i, 4);

    
    if estado == 0
        color_fondo = colores(1, :);
    elseif estado == -1
        color_fondo = colores(2, :);
    elseif estado == 1
        color_fondo = colores(3, :);
    end

    patch([x_inicio, x_fin, x_fin, x_inicio], ...
      [-5, -5, 5, 5], ...
      color_fondo, 'EdgeColor', 'none', 'FaceAlpha', 0.3);

end

hold on

plot(T_total, fuerza_r_externa, 'b', 'LineWidth', 1.5, 'DisplayName', 'External force (= uFr)' );
plot(T_total, F_tot_eje_cuerda_sin_rozamiento, 'r:', 'LineWidth', 1.5, 'DisplayName', 'F without friction' );
plot(T_total, F_tot_eje_cuerda, 'g:', 'LineWidth', 1.5, 'DisplayName', 'F final rope axis' );

title('Forces on the mass in the rope axis');
ylabel('Forces (N)');
xlabel('Time (s)');


h_legends = findobj(gca, '-regexp', 'DisplayName', '.+');
legend(h_legends);

grid on;


% Título general para toda la figura
sgtitle('FORCES (Yellow = stopped, Green = +, Red = -)', 'FontSize', 14, 'FontWeight', 'bold');




%%%

%%%%%%%%%%%%%%%%%%%%%
%% ..:: State variable graphs: ::..
%%%%%%%%%%%%%%%%%%%%%


figure(501);
sgtitle('STATE VARIABLES', 'FontSize', 14, 'FontWeight', 'bold');


variables = {'y_{trolley}', 'dy_{trolley}', 'x_{trolley}', 'dx_{trolley}', 'alpha', ...
             'dalpha', 'beta', 'dbeta', 'R', 'dR'};


for i = 1:10
    subplot(4, 3, i); 
    plot(T_total, X_total(:, i), 'LineWidth', 1.5); 
    ylabel(variables{i}, 'Interpreter', 'tex'); 
    grid on; 
    if i > 7 
        xlabel('Time (s)');
    end
end


for i = 11:12
    subplot(4, 3, i);
    axis off; 
end


linkaxes(findall(gcf, 'Type', 'axes'), 'x');


% :::::::::  Payload position graphs: ::::::::::::::::::::::::::::::

figure(502);

variables_rel = {'x_{relative} (m)', 'y_{relative} (m)', 'z_{relative} (m)'};

variables_abs = {'x_{abs} (m)', 'y_{abs} (m)', 'z_{abs} (m)'};

for i = 1:3
    subplot(3, 2, 2*i-1); 
    plot(T_total, pos_rel_carga(:, i), 'LineWidth', 1.5); 
    ylabel(variables_rel{i}, 'Interpreter', 'tex');
    grid on; 
    if i == 3
        xlabel('Time (s)'); 
    end
    if i == 1
        title('Relative payload positions', 'FontWeight', 'bold'); 
    end
end

for i = 1:3
    subplot(3, 2, 2*i); 
    plot(T_total, pos_abs_carga(:, i), 'LineWidth', 1.5); 
    ylabel(variables_abs{i}, 'Interpreter', 'tex'); 
    grid on; 
    if i == 3
        xlabel('Time (s)'); 
    end
    if i == 1
        title('Absolute payload positions', 'FontWeight', 'bold'); 
    end
end


sgtitle('PAYLOAD POSITIONS', 'FontSize', 14, 'FontWeight', 'bold');

% Vincular los ejes X de todos los gráficos
linkaxes(findall(gcf, 'Type', 'axes'), 'x');





%%%%%%%%%%%%%%%%%%%%%
%% ..:: 3D FIGURE ::..
%%%%%%%%%%%%%%%%%%%%%
    % Input data
    Tgraf = 0.01; % Desired sampling period for plotting in seconds
    % Desired time vector
    T_desired = 0:Tgraf:T_total(end);
    % Initialize the resulting vectors
    T_result = zeros(size(T_desired));
    x_carro = zeros(size(T_desired)); % x_trolley column
    y_carro = zeros(size(T_desired)); % y_trolley column
    z_carro = zeros(size(T_desired)); % Assuming z_trolley = 0 (XY plane)
    x_carga = zeros(size(T_desired));
    y_carga = zeros(size(T_desired));
    z_carga = zeros(size(T_desired));
    % Find the values immediately preceding each T_desired
    for i = 1:length(T_desired)
        % Find the index of the maximum value that is less than or equal to the desired one
        idx = find(T_total <= T_desired(i), 1, 'last');
        % Store the corresponding time and solution
        T_result(i) = T_total(idx);
        x_carro(i) = X_total(idx,3);
        y_carro(i) = X_total(idx,1);
        x_carga(i) = pos_abs_carga(idx,1);
        y_carga(i) = pos_abs_carga(idx,2);
        z_carga(i) = pos_abs_carga(idx,3);
    end
    % Create figure
    figure(503);
    hold on;
    grid on;
    xlabel('X (m)');
    ylabel('Y (m)');
    zlabel('Z (m)');
    title('3D Evolution of the Trolley and Load', 'FontSize', 14, 'FontWeight', 'bold');
    view(3);
    axis equal;
    % ::: representation of the static part of the gantry crane
    % Define the coordinates of the square with length 1x1 centered at (0,0,0)
    x_square = [0, 0.507, 0.507, 0];
    y_square = [0, 0, 0.667, 0.667];
    z_square = [0, 0, 0, 0];  % All points on the z = 0 plane
    % Create the fill for the square (transparent red)
    fill3(x_square, y_square, z_square, 'r', 'FaceAlpha', 0.1);  % Red color with transparency
    % Draw the borders of the square (black and thick)
    plot3([x_square, x_square(1)], [y_square, y_square(1)], [z_square, z_square(1)], 'k', 'LineWidth', 3);
    % Draw thick lines from each corner towards z = -1
    for i = 1:4
        plot3([x_square(i), x_square(i)], [y_square(i), y_square(i)], [z_square(i), -1], 'k', 'LineWidth', 3);
    end
    % Set fixed limits for the axes
    xlim([-0.1, 0.7]);
    ylim([-0.1, 0.7]);
    zlim([-1, 0.2]);
    % ::: rail representation :::
    % Initialize the semi-transparent black line
    y_line = [0, 0.667]; % Centered at y = 0, length = 1
    z_line = [0, 0];      % On the z = 0 plane
    x_line = [x_carro(1), x_carro(1)]; % Initial X coordinate same as the trolley's
    % Create the semi-transparent black line
    linea_negra_carril = plot3(x_line, y_line, z_line, 'k-', 'LineWidth', 3, 'Color', [0, 0, 0, 0.5]);
    %:::
    % Initialize plots
    carro_trayectoria = plot3(x_carro(1), y_carro(1), z_carro(1), 'r-', 'LineWidth', 1.5);
    carga_trayectoria = plot3(x_carga(1), y_carga(1), z_carga(1), 'b-', 'LineWidth', 1.5);
    carro_punto = plot3(x_carro(1), y_carro(1), z_carro(1), 'ro', 'MarkerSize', 8, 'MarkerFaceColor', 'r');
    carga_punto = plot3(x_carga(1), y_carga(1), z_carga(1), 'go', 'MarkerSize', 8, 'MarkerFaceColor', 'g');
    % Line connecting the trolley and the load
    conexion = plot3([x_carro(1), x_carga(1)], [y_carro(1), y_carga(1)], [z_carro(1), z_carga(1)], ...
        'k--', 'LineWidth', 1.2);
    % Draw local axes
    carro_ejes = quiver3(0, 0, 0, 0, 0, 0, 'r', 'LineWidth', 1.5);
    carga_ejes = quiver3(0, 0, 0, 0, 0, 0, 'g', 'LineWidth', 1.5);
    % Local axis parameters (length)
    longitud_ejes = 0.5;
    % Create the text label to display the time
    h_time_label = text(0.95, 0.95, 1.05, ['Time: ', num2str(0)], 'Units', 'normalized', 'FontSize', 12, 'HorizontalAlignment', 'right');
    % Animate the evolution
    for k = 1:2:length(T_result)  % NOTE: I'VE SET THE STEP TO 2 TO MAKE IT GO FASTER
        % Update trajectories
        set(carro_trayectoria, 'XData', x_carro(1:k), 'YData', y_carro(1:k), 'ZData', z_carro(1:k));
        set(carga_trayectoria, 'XData', x_carga(1:k), 'YData', y_carga(1:k), 'ZData', z_carga(1:k));
        % Update point positions
        set(carro_punto, 'XData', x_carro(k), 'YData', y_carro(k), 'ZData', z_carro(k));
        set(carga_punto, 'XData', x_carga(k), 'YData', y_carga(k), 'ZData', z_carga(k));
        % Update connection line between trolley and load
        set(conexion, 'XData', [x_carro(k), x_carga(k)], ...
            'YData', [y_carro(k), y_carga(k)], ...
            'ZData', [z_carro(k), z_carga(k)]);
        % Update trolley's local axes
        set(carro_ejes, 'XData', [x_carro(k), x_carro(k)], 'YData', [y_carro(k), y_carro(k)], ...
            'ZData', [z_carro(k), z_carro(k)], ...
            'UData', [longitud_ejes, 0], 'VData', [0, longitud_ejes], 'WData', [0, 0]);
        % Update load's local axes
        set(carga_ejes, 'XData', [x_carga(k), x_carga(k)], 'YData', [y_carga(k), y_carga(k)], ...
            'ZData', [z_carga(k), z_carga(k)], ...
            'UData', [longitud_ejes, 0], 'VData', [0, longitud_ejes], 'WData', [0, 0]);
        % Update the time label
        set(h_time_label, 'String', ['Time (s): ', num2str(T_result(k))]);
        % Update the rail's position
        set(linea_negra_carril, 'XData', [x_carro(k), x_carro(k)], ...
                         'YData', y_line, ...
                         'ZData', z_line);
        % Pause for animation
        %t_por_frame = T_total(end)/numel(T_total);
        pause(Tgraf);
    end
    %
% %%%%%%%%%%%%%%%%%%%%%
% %% END 3D FIGURE
% %%%%%%%%%%%%%%%%%%%%%




%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% FUNCTION OUTPUT
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

T_out = T_total;

DATA_OUT = [X_total(:,1) X_total(:,2) ddyw_vec X_total(:,3) X_total(:,4) ddxw_vec X_total(:,5) X_total(:,6) X_total(:,7) X_total(:,8) X_total(:,9) X_total(:,10) ddR_vec y_abs_carga dyc ddyc x_abs_carga dxc ddxc z_rel_carga dzc ddzc];

ESTADO_OUT = registro_q_expandido;

end
