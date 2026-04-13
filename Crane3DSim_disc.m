%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%       J.VICENTE - j.vicente@unizar.es           LAST UPDATE: 13/04/2026
%       [V2.0]
%       Universidad de Zaragoza - Instituto de Investigacion en Ingenieria
%       de Aragon
%
%  ----------------------------------------------------------------------------
%  ----------------------------------------------------------------------------
%
%  ------ WHAT'S NEW? ---------------------------------------------------------
%
% V2.0 -- Fixed problems with oscilation friction terms on the X and Y trolley dynamics
%      -- Fixed a BUG on the rope axis checks after an event
%
%
%  ---------------------------------------------------------------------------
% 
% Function to simulate the dynamics of the 3D crane bridge with discrete
% control signal and continuous (ODE) system.
%
%
%
%  ----------------------------------------------------------------------------
%  ----------------------------------------------------------------------------
%
% function [T_out DATA_OUT ESTADO_OUT] = Crane3DSim_disc(fricciones,IMOT,IMOTx,IMOTy,ks,uPWM_x_matriz,uPWM_y_matriz,uPWM_r_matriz,masas,x0,tsimul,T_sample,g,ph_limits,refs)
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
%   uPWM_x_matriz =  x-axis force input (PWM from 0 to 1) must be a column
%   vercor with the force every T_sample
%   uPWM_y_matriz =  y-axis fforce input (PWM from 0 to 1) must be a column
%   vercor with the force every T_sample
%   uPWM_z_matriz =  z-axis force input (PWM from 0 to 1) must be a column
%   vercor with the force every T_sample
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
% > T_sample = sampling time
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
% > Rerefences for Feedback control
%    refs -- matrix to insert the references if they are not calculated in the function itself. 
%            It can be used as desired as long as it is then used consistently in the calculation block 
%            of the feedback action. In this example, references are inserted for the position of the trolley 
%           (first column on the X axis, second on the Y axis, third on the length of the rope). 
%           Each row is the reference for each instant, discretized with T_sample.
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


function [T_out DATA_OUT ESTADO_OUT] = Crane3DSim_disc(frictions,IMOT,IMOTx,IMOTy,ks,uPWM_x_matriz,uPWM_y_matriz,uPWM_r_matriz,masas,x0,tsimul,T_sample,g,ph_limits,refs)


%% CTES PWM - N

X_PWM_TO_F = ks(1);
Y_PWM_TO_F = ks(2);
Z_PWM_TO_F = ks(3);

% g = 9.81;   % gravity [m/s2]


%% PHYSICAL LIMITS 

xlim_positivo = ph_limits(1);
xlim_negativo = ph_limits(2);
ylim_positivo = ph_limits(3);
ylim_negativo = ph_limits(4);
rlim_positivo = ph_limits(5);
rlim_negativo = ph_limits(6);  


%% FRICTION

Tsx_fun_negativo = @(x) frictions.Tsx_negative(x)*X_PWM_TO_F;  
Tsx_fun_positivo = @(x) frictions.Tsx_positive(x)*X_PWM_TO_F;  

Tsy_fun_positivo = @(y) frictions.Tsy_positive(y)*Y_PWM_TO_F;   
Tsy_fun_negativo = @(y) frictions.Tsy_negative(y)*Y_PWM_TO_F;    

Tsr_fun_positivo = @(l) frictions.Tsr_positive(l)*Z_PWM_TO_F;   
Tsr_fun_negativo = @(l) frictions.Tsr_negative(l)*Z_PWM_TO_F;   

Tdy_positivo = frictions.Tdy_positive*Y_PWM_TO_F;  
Tdy_negativo = frictions.Tdy_negative*Y_PWM_TO_F;  

Tdx_positivo = frictions.Tdx_positive*X_PWM_TO_F; 
Tdx_negativo = frictions.Tdx_negative*X_PWM_TO_F; 

Tdr_positivo = frictions.Tdr_positive*Z_PWM_TO_F;   
Tdr_negativo = frictions.Tdr_negative*Z_PWM_TO_F;  

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

uN_x_matriz = uPWM_x_matriz*X_PWM_TO_F;%1*ones(size(matriz_salida_X,1),1);
uN_y_matriz = uPWM_y_matriz*Y_PWM_TO_F;%zeros(size(matriz_salida_Y,1),1);
uN_r_matriz = uPWM_r_matriz*Z_PWM_TO_F;%ones(size(matriz_salida_Z,1),1);


qx = 0;
qy = 0;
qr = 0;




%% VARIABLES TO STORE RESULTS

T_total = [];
X_total = [];
U_aplicadas_total = []; %[N]
registro_q = [];

registro_uPWMx = [] ; %TOTAL, FF, FB [PWM]
registro_uPWMy = [] ; %TOTAL, FF, FB [PWM]
registro_uPWMr = [] ; %TOTAL, FF, FB [PWM]


%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%  - SIMULATION AND CONTROL LOOP - 
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

% The sampling time is an input to the function


%% SIMULATION + SAMPLING SECTIONS

tfin_largo = tf+1; 
ts_inicio = t0:T_sample:(tfin_largo-T_sample);
ts_fin = ts_inicio + T_sample;
tramos = [ts_inicio' ts_fin'];

icontrol = 1; % To go through the vector with the FF actions


%%%%% INITIALIZATION OF THE EXAMPLE PI %%%%% 

    % --> PI TROLLEY ON X-AXIS
    i1_k1 = 0;
    err_xw_k1 = 0;
    Kc1 = 100;%10;%2000;
    Ti1 = 5;%0.5;
    
    % --> PI TROLLEY ON Y-AXIS
    i2_k1 = 0;
    err_yw_k1 = 0;
    Kc2 = 100;%10;%2000;
    Ti2 = 5;%0.5;
    
    % --> PI ROPE
    i3_k1 = 0;
    err_l_k1 = 0;
    Kc3 = 100;%10;%2000;
    Ti3 = 5;%0.5;


%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%


for i = 1:size(tramos,1)
    tspan = tramos(i,:);
    tactual = tspan(1);



    % stop the simulation if the time is over
    if (tf-tactual) < 1e-10
        disp("COMPLETED PATH");
        break;
    end

    X_anterior = x;


    %%%%% >> FEEDBACK CONTROL << %%%%%
    % Example with a PI on the trolley position

        % 1. Reading the signals
            x_carro_leido = x(3);
            y_carro_leido = x(1);
            lcuerda_leido = x(9);
            alpha_leido = x(5);
            beta_leido = x(7);

        % 2. Error calculation
            err_xw_k = refs(icontrol,1) - x_carro_leido;
            err_yw_k = refs(icontrol,2) - y_carro_leido;
            err_l_k = refs(icontrol,3) - lcuerda_leido;


        % 3. Action calculation
            
        % PI 1 --> eje X
        
        i1_k = i1_k1+T_sample*((err_xw_k+err_xw_k1)/2);
        uF_x_FB = Kc1 * err_xw_k + Kc1/Ti1 * i1_k;
        err_xw_k1 = err_xw_k;
        i1_k1 = i1_k;
        
        % PI 2 --> eje Y
        
        i2_k = i2_k1+T_sample*((err_yw_k+err_yw_k1)/2);
        uF_y_FB = Kc2 * err_yw_k + Kc2/Ti2 * i2_k;
        err_yw_k1 = err_yw_k;
        i2_k1 = i2_k;
        
        % PI 3 --> eje l
        
        i3_k = i3_k1+T_sample*((err_l_k+err_l_k1)/2);
        uF_r_FB = Kc3 * err_l_k + Kc3/Ti3 * i3_k;
        err_l_k1 = err_l_k;
        i3_k1 = i3_k;
        
        
   %%%%% >> FEEDFORWARD CONTROL << %%%%%       
        
        uF_x_FF = uN_x_matriz(icontrol);
        uF_y_FF = uN_y_matriz(icontrol);
        uF_r_FF = uN_r_matriz(icontrol);



   %%%%% >> SUM OF CONTROL ACTIONS << %%%%%
        % 
        % uF_x_aplicar_p = (uF_x_FF+uF_x_FB);
        % uF_y_aplicar_p = (uF_y_FF+uF_y_FB);
        % uF_r_aplicar_p = (uF_r_FF+uF_r_FB);

        uF_x_aplicar_p = (uF_x_FF);
        uF_y_aplicar_p = (uF_y_FF);
        uF_r_aplicar_p = (uF_r_FF);


        max_action_x = 1*X_PWM_TO_F;
        min_action_x = -1*X_PWM_TO_F;

        max_action_y = 1*Y_PWM_TO_F;
        min_action_y = -1*Y_PWM_TO_F;

        max_action_r = 1*Z_PWM_TO_F;
        min_action_r = -1*Z_PWM_TO_F;
        
        uF_x_aplicar = max(min_action_x, min(uF_x_aplicar_p, max_action_x));
        uF_y_aplicar = max(min_action_y, min(uF_y_aplicar_p, max_action_y));
        uF_r_aplicar = max(min_action_r, min(uF_r_aplicar_p, max_action_r));

        registro_uPWMx = [registro_uPWMx; tactual uF_x_aplicar uF_x_FF uF_x_FB];
        registro_uPWMy = [registro_uPWMy; tactual uF_y_aplicar uF_y_FF uF_y_FB];
        registro_uPWMr = [registro_uPWMr; tactual uF_r_aplicar uF_r_FF uF_r_FB];

   
    icontrol = icontrol+1;

    

        Tdiscret = 0.002;  
    
        [qx_new, qy_new, qr_new, X_k1, T_k1] = Crane3DSim_1state_fast(Tdiscret, X_anterior, uF_x_aplicar, uF_y_aplicar , uF_r_aplicar , mw, ms, mc, Tsx_fun_positivo, Tsx_fun_negativo , Tsy_fun_positivo, Tsy_fun_negativo, Tsr_fun_positivo, Tsr_fun_negativo, Tdy_positivo,Tdy_negativo, Tdx_positivo, Tdx_negativo, Tdr_positivo, Tdr_negativo, g, K_AIREA, K_AIREB, xlim_positivo, xlim_negativo, ylim_positivo, ylim_negativo, qx, qy, qr,IMOT, IMOTx, IMOTy);
    
    
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

    
        
    while tspan(1)<tspan(2)  

        aux_registro_q = [tspan(1) qx qy qr];
        registro_q = [registro_q; aux_registro_q];

        % EVENTS FUNCTION:
        funcion_eventos = @(t, x) EventsFnc(t, x, uF_x_aplicar, uF_y_aplicar, uF_r_aplicar, qx, qy, qr, Tsx_fun_positivo, Tsx_fun_negativo , Tsy_fun_positivo, Tsy_fun_negativo, Tsr_fun_positivo, Tsr_fun_negativo, Tdy_positivo,Tdy_negativo, Tdx_positivo, Tdx_negativo, Tdr_positivo, Tdr_negativo , g, mc,mw,ms, xlim_positivo, xlim_negativo, ylim_positivo,ylim_negativo,rlim_positivo,rlim_negativo) ; 
    
        % DYNAMIC FUNCTION:
        funcion_ode = @(t, x) func_modo1(t, x, uF_x_aplicar, uF_y_aplicar, uF_r_aplicar, mw, ms, mc, Tsx_fun_positivo, Tsx_fun_negativo , Tsy_fun_positivo, Tsy_fun_negativo, Tsr_fun_positivo, Tsr_fun_negativo, Tdy_positivo,Tdy_negativo, Tdx_positivo, Tdx_negativo, Tdr_positivo, Tdr_negativo, g, K_AIREA, K_AIREB, qx, qy, qr);
    
        % Solving the differential equation
        options = odeset('Events', funcion_eventos, 'AbsTol', 1e-13, 'RelTol',1e-13);% 'MaxStep',1e-4);       
        
        [T, X_new, TE, YE, IE] = ode45(funcion_ode, tspan, x, options);        
            
        % Saving the results
        T_total = [T_total; T];
        X_total = [X_total; X_new];
    
        aux_u_aplicadas = ones(size(T,1),3);
        aux_u_aplicadas(:,1) = uF_x_aplicar;
        aux_u_aplicadas(:,2) = uF_y_aplicar;
        aux_u_aplicadas(:,3) = uF_r_aplicar;
        
        U_aplicadas_total = [U_aplicadas_total; aux_u_aplicadas];
        
        % Updating the time
        tspan(1) = T(end);
    
        % New initial state
        x = X_new(end, :);    
    
        % Calculate the force being done by the rope
    
                uF_x_apl = uF_x_aplicar;
                uF_y_apl = uF_y_aplicar;
                uF_r_apl = uF_r_aplicar;
      
           
        if qr == 0                                                              % Does not move on the axis of the rope
            S = - g*mc*(cos(x(7))*sin(x(5)));        
        elseif qr == 1                                                          % Moves upward x(10)<0
            Tr = Tdr_positivo*x(10) - Tsr_fun_positivo(x(9));                             
            S = uF_r_apl - Tr;
        elseif qr == -1                                                         % Moves downward x(10)>0    
            Tr = Tdr_negativo*x(10) + Tsr_fun_negativo(x(9));                             
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
                    if (((uF_x_apl - sin(x(5))*sin(x(7))*S + ((K_AIREA*x(6)*cos(x(5))*sin(x(7)) + K_AIREB*x(8)*sin(x(5))*cos(x(7)))/x(9))) > Tsx_fun_positivo(x(3))) && (x(3)<xlim_positivo))             % dont stand still because I move from -x to +x.
                        qx = 1;
                    elseif (((uF_x_apl - sin(x(5))*sin(x(7))*S + ((K_AIREA*x(6)*cos(x(5))*sin(x(7)) + K_AIREB*x(8)*sin(x(5))*cos(x(7)))/x(9))) < -Tsx_fun_negativo(x(3))) && (x(3)>xlim_negativo))        % dont stand still because I move from +x to -x.
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
                    if (((uF_y_apl -S*cos(x(5)) - ((K_AIREA*x(6)*sin(x(5)))/x(9))  ) > Tsy_fun_positivo(x(1))) && (x(1)<ylim_positivo))              % dont stand still because  move from -y to +y.
                        qy = 1;
                    elseif (((uF_y_apl -S*cos(x(5)) - ((K_AIREA*x(6)*sin(x(5)))/x(9))) < -Tsy_fun_negativo(x(1))) && (x(1)>ylim_negativo))         % dont stand still because  move from +y to -y.
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
                        dx(4) =  (uF_x_apl -(Tdx_positivo*x(4) + Tsx_fun_positivo(x(3))) - S*sin(x(5))*sin(x(7)) + ((K_AIREA*x(6)*cos(x(5))*sin(x(7)) + K_AIREB*x(8)*sin(x(5))*cos(x(7)))/x(9)) )/(ms + mw + IMOTx);                    
                    elseif qx == -1             
                        dx(4) =  (uF_x_apl -(Tdx_negativo*x(4) - Tsx_fun_negativo(x(3))) - S*sin(x(5))*sin(x(7)) + ((K_AIREA*x(6)*cos(x(5))*sin(x(7)) + K_AIREB*x(8)*sin(x(5))*cos(x(7)))/x(9)))/(ms + mw + IMOTx);                 
                    else
                        dx(4) = 0;                    
                    end
    
    
                    % -- y --
    
                    if qy == 1                  
                        dx(2) = (uF_y_apl - (Tdy_positivo*x(2) + Tsy_fun_positivo(x(1))) - S*cos(x(5)) - ((K_AIREA*x(6)*sin(x(5)))/x(9)) )/(mw + IMOTy);                    
                    elseif qy == -1             
                        dx(2) = (uF_y_apl - (Tdy_negativo*x(2) - Tsy_fun_negativo(x(1))) - S*cos(x(5)) - ((K_AIREA*x(6)*sin(x(5)))/x(9)) )/(mw + IMOTy);             
                         
                    else
                        dx(2) = 0;
                        
                    end
    
    
                    % -- rope --
    
                    S1 = uF_r_apl;
    
    
                    f_cuerda = (S1 - mc*cos(x(5))*dx(2) + mc*x(9)*x(6)^2 + mc*x(9)*x(8)^2 - mc*cos(x(5))^2*x(9)*x(8)^2 - mc*sin(x(5))*sin(x(7))*dx(4) + g*mc*cos(x(7))*sin(x(5)) );
    
                   
                    if (f_cuerda < -Tsr_fun_positivo(x(9))) && (x(9)>rlim_negativo)                             
                        qr = 1;
                    elseif (f_cuerda > Tsr_fun_negativo(x(9))) && (x(9)<rlim_positivo)                           
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
        U_aplicadas_total = U_aplicadas_total(idx,:);
    end
end

% I save the last state in which it is in each axis.
aux_registro_q = [T_total(end) qx qy qr];
registro_q = [registro_q; aux_registro_q];



%% DYNAMIC FUNCTION

function dx_dt = func_modo1(t, x, uF_x_aplicar, uF_y_aplicar, uF_r_aplicar, mw, ms, mc, Tsx_fun_positivo, Tsx_fun_negativo , Tsy_fun_positivo, Tsy_fun_negativo, Tsr_fun_positivo, Tsr_fun_negativo, Tdy_positivo,Tdy_negativo, Tdx_positivo, Tdx_negativo, Tdr_positivo, Tdr_negativo, g, K_AIREA, K_AIREB, qx, qy, qr)
 

%% ROPE TENSION
    if qr == 0                                                              % No movement along the rope's axis
        S =  -g*mc*(cos(x(7))*sin(x(5)));
    elseif qr == 1                                                          % Rope moves upwards                                                         
        Tr = Tdr_positivo*x(10) - Tsr_fun_positivo(x(9));                             % upwards, x(10)<0 and the total force must be <0    
        S = uF_r_aplicar-Tr;       
    elseif qr == -1                                                         % It moves downwards (negative velocity)        
        Tr = Tdr_negativo*x(10) + Tsr_fun_negativo(x(9));                             % downwards, x(10)>0 and the total force must be >0    
        S = uF_r_aplicar-Tr;       
    end     

%% X-AXIS
    if qx == 0                                                              % No movement in x
        Tx = uF_x_aplicar;
        dx(3) = 0;                                                          % dxw    
        dx(4) = 0;                                                          % ddxw        
    elseif qx == 1                                                          % It moves in the +x direction (positive velocity)
        Tx = Tdx_positivo*x(4) + Tsx_fun_positivo(x(3));                              % [N] (absolute value, with direction but no sign)
        dx(3) = x(4);                                                       % dxw            
        dx(4) =  (uF_x_aplicar -Tx - S*sin(x(5))*sin(x(7)) + ((K_AIREA*x(6)*cos(x(5))*sin(x(7)) + K_AIREB*x(8)*sin(x(5))*cos(x(7)))/x(9))  ) /  (ms + mw + IMOTx);           % ddxw
    elseif qx == -1                                                         % It moves in the -x direction (negative velocity)
        Tx = Tdx_negativo*x(4) - Tsx_fun_negativo(x(3));                              % [N] (absolute value, with direction but no sign)
        dx(3) = x(4);                                                       % dxw         
        dx(4) =  (uF_x_aplicar -Tx - S*sin(x(5))*sin(x(7)) + ((K_AIREA*x(6)*cos(x(5))*sin(x(7)) + K_AIREB*x(8)*sin(x(5))*cos(x(7)))/x(9))  ) /  (ms + mw + IMOTx);           % ddxw
    end

%% Y-AXIS
    if qy == 0                                                              % No movement in y
        Ty = uF_y_aplicar;
        dx(1) = 0;                                                          % dyw    
        dx(2) = 0;                                                          % ddyw        
    elseif qy == 1                                                          % It moves in the +y direction (positive velocity)
        Ty = Tdy_positivo*x(2) + Tsy_fun_positivo(x(1));                              % [N] (absolute value, with direction but no sign)
        dx(1) = x(2);                                                       % dyw
        dx(2) = (uF_y_aplicar - Ty - S*cos(x(5)) - ((K_AIREA*x(6)*sin(x(5)))/x(9))  )/(mw + IMOTy);                            % ddyw 
    elseif qy == -1                                                         % It moves in the -y direction (negative velocity)
        Ty = Tdy_negativo*x(2) -Tsy_fun_negativo(x(1));                               % [N] (absolute value, with direction but no sign)
        dx(1) = x(2);                                                       % dyw
        dx(2) = (uF_y_aplicar - Ty - S*cos(x(5)) - ((K_AIREA*x(6)*sin(x(5)))/x(9))  )/(mw + IMOTy);                            % ddyw 
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


dx(8) = (1/ (sin(x(5))*x(9))) * (-g*sin(x(7)) -cos(x(7))*dx(4) -2*sin(x(5))*x(10)*x(8) - 2*cos(x(5))*x(9)*x(6)*x(8) )  - 1/(mc*x(9)^2*sin(x(5))^2)*K_AIREB*x(8); % ddBETA




dx_dt = transpose(dx);

end


%% EVENTS FUNCTION

function [value, isterminal, direction] = EventsFnc(t, x, uF_x_aplicar, uF_y_aplicar, uF_r_aplicar, qx, qy, qr, Tsx_fun_positivo, Tsx_fun_negativo , Tsy_fun_positivo, Tsy_fun_negativo, Tsr_fun_positivo, Tsr_fun_negativo, Tdy_positivo,Tdy_negativo, Tdx_positivo, Tdx_negativo, Tdr_positivo, Tdr_negativo , g, mc,mw,ms, xlim_positivo, xlim_negativo, ylim_positivo,ylim_negativo,rlim_positivo,rlim_negativo)
   
    % Calculate the force exerted by the rope
    % S = uF_r(t);
    
    if qr == 0                                                              % No movement along the rope's axis
        S = - g*mc*(cos(x(7))*sin(x(5)));
    elseif qr == 1                                                          % It moves upwards, x(10)<0
        % [N] (absolute value, with direction but no sign) For the rope, when it extends, r_dot is positive,
        % but the positive force is defined to shorten the rope.
        Tr = Tdr_positivo*x(10) - Tsr_fun_positivo(x(9));
        S = uF_r_aplicar - Tr;
    elseif qr == -1                                                         % It moves downwards, x(10)>0
        % [N] (absolute value, with direction but no sign) For the rope, when it extends, r_dot is positive,
        % but the positive force is defined to shorten the rope.
        Tr = Tdr_negativo*x(10) + Tsr_fun_negativo(x(9));
        S = uF_r_aplicar - Tr;
    end
    
    % Here I want to detect that the forces are greater than the
    % dry friction for the movement to start.

    %% X-AXIS
    
    % MOVEMENT TOWARDS +X (uF_x(t) will be positive); when it changes from - to +,
    % it starts moving towards +X (here, only the - to + change is detected)
    f1x = ((uF_x_aplicar - sin(x(5))*sin(x(7))*S + ((K_AIREA*x(6)*cos(x(5))*sin(x(7)) + K_AIREB*x(8)*sin(x(5))*cos(x(7)))/x(9)) ) - Tsx_fun_positivo(x(3))) * (x(3)<xlim_positivo);
    % disp(f1x)
    
    % MOVEMENT TOWARDS -X (uF_x(t) will be negative); when it changes from + to -,
    % it starts moving towards -X (here, only the + to - change is detected)
    f2x = ((uF_x_aplicar - sin(x(5))*sin(x(7))*S + ((K_AIREA*x(6)*cos(x(5))*sin(x(7)) + K_AIREB*x(8)*sin(x(5))*cos(x(7)))/x(9)) ) + Tsx_fun_negativo(x(3))) * (x(3)>xlim_negativo);
    
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
    f4x = xlim_positivo-x(3);
    d4x = -1;
    
    % DETECT THE POSITION TO STOP IF X <= MINIMUM X LIMIT
    f5x = xlim_negativo-x(3);
    d5x = 1;
    
    %% Y-AXIS
    
    % MOVEMENT TOWARDS +Y (uF_y(t) will be positive); when it changes from - to +,
    % it starts moving towards +Y (here, only the - to + change is detected)
    f1y = ((uF_y_aplicar -S*cos(x(5)) - ((K_AIREA*x(6)*sin(x(5)))/x(9))) - Tsy_fun_positivo(x(1))) * (x(1)<ylim_positivo);
    
    % MOVEMENT TOWARDS -Y (uF_y(t) will be negative); when it changes from + to -,
    % it starts moving towards -Y (here, only the + to - change is detected)
    f2y = ((uF_y_aplicar -S*cos(x(5)) - ((K_AIREA*x(6)*sin(x(5)))/x(9))) + Tsy_fun_negativo(x(1))) * (x(1)>ylim_negativo);
    
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
    f4y = ylim_positivo-x(1);
    d4y = -1;
    
    % DETECT THE POSITION TO STOP IF Y <= MINIMUM Y LIMIT
    f5y = ylim_negativo-x(1);
    d5y = 1;
    
    %% ROPE AXIS
    
    if qx == 1                  % it moves in +X
        dx(4) =  (uF_x_aplicar - (Tdx_positivo*x(4) + Tsx_fun_positivo(x(3))) - S*sin(x(5))*sin(x(7)) + ((K_AIREA*x(6)*cos(x(5))*sin(x(7)) + K_AIREB*x(8)*sin(x(5))*cos(x(7)))/x(9)) )/(ms + mw + IMOTx);
    elseif qx == -1             % it moves in -X
        dx(4) =  (uF_x_aplicar - (Tdx_negativo*x(4) - Tsx_fun_negativo(x(3))) - S*sin(x(5))*sin(x(7)) + ((K_AIREA*x(6)*cos(x(5))*sin(x(7)) + K_AIREB*x(8)*sin(x(5))*cos(x(7)))/x(9)) )/(ms + mw + IMOTx);
    else
        dx(4) = 0;
    end
    % -- y --
    if qy == 1                  % it moves in +Y
        dx(2) = (uF_y_aplicar - (Tdy_positivo*x(2) + Tsy_fun_positivo(x(1))) - S*cos(x(5)) - ((K_AIREA*x(6)*sin(x(5)))/x(9)) )/(mw + IMOTy);
    elseif qy == -1             % it moves in -Y
        dx(2) = (uF_y_aplicar - (Tdy_negativo*x(2) - Tsy_fun_negativo(x(1))) - S*cos(x(5)) - ((K_AIREA*x(6)*sin(x(5)))/x(9)) )/(mw + IMOTy);
    else
        dx(2) = 0;
    end
    % -- rope --
    S1 = uF_r_aplicar;
    f_cuerda = (S1 - mc*cos(x(5))*dx(2) + mc*x(9)*x(6)^2 + mc*x(9)*x(8)^2 - mc*cos(x(5))^2*x(9)*x(8)^2 - mc*sin(x(5))*sin(x(7))*dx(4) + g*mc*cos(x(7))*sin(x(5)) );
    
    % MOVEMENT UPWARDS (ftot_rope will be negative); when it changes from + to -,
    % it starts moving upwards (here, only the + to - change is detected)
    f1r = f_cuerda + Tsr_fun_positivo(x(9));
    %f1r = ftot_cuerda_hacia_arriba;
    
    % MOVEMENT DOWNWARDS (ftot_rope will be positive); when it changes from - to +,
    % it starts moving downwards (here, only the - to + change is detected)
    f2r = f_cuerda - Tsr_fun_negativo(x(9));
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
    f4r = rlim_positivo-x(9);
    d4r = -1;
    
    % DETECT THE POSITION TO STOP IF ROPE LENGTH <= MINIMUM ROPE LENGTH LIMIT
    f5r = rlim_negativo-x(9);
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

S_vec_pos = zeros(size(dzc));


for i = 1:size(T_total,1)
    x = X_total(i,:);
    t = T_total(i);
    qx = registro_q_expandido(i,1);
    qy = registro_q_expandido(i,2);
    qr = registro_q_expandido(i,3);

    uF_x_aplicar = U_aplicadas_total(i,1);
    uF_y_aplicar = U_aplicadas_total(i,2);
    uF_r_aplicar = U_aplicadas_total(i,3);

    if qr == 0                                                             
        S = - g*mc*(cos(x(7))*sin(x(5)));        
    elseif qr == 1                                                          
        Tr = Tdr_positivo*x(10) - Tsr_fun_positivo(x(9));                   
        S = uF_r_aplicar - Tr;
    elseif qr == -1                                                         
        Tr = Tdr_negativo*x(10) + Tsr_fun_negativo(x(9));                   
        S = uF_r_aplicar - Tr;
    end 

    S_vec_pos(i) = S;


    %% X-AXIS
    
    if qx == 0                                                              
        ddxw = 0;
    elseif qx == 1                                                          
        Tx = Tdx_positivo*x(4) + Tsx_fun_positivo(x(3));                             
        ddxw =  (uF_x_aplicar -Tx - S*sin(x(5))*sin(x(7)) + ((K_AIREA*x(6)*cos(x(5))*sin(x(7)) + K_AIREB*x(8)*sin(x(5))*cos(x(7)))/x(9)) )/(ms + mw + IMOTx);             
    elseif qx == -1                                                        
        Tx = Tdx_negativo*x(4) - Tsx_fun_negativo(x(3));                                  
        ddxw =  (uF_x_aplicar -Tx - S*sin(x(5))*sin(x(7)) + ((K_AIREA*x(6)*cos(x(5))*sin(x(7)) + K_AIREB*x(8)*sin(x(5))*cos(x(7)))/x(9)) )/(ms + mw + IMOTx);           
    end


    %% Y-AXIS

    if qy == 0                                                              
        ddyw = 0;
    elseif qy == 1                                                          
        Ty = Tdy_positivo*x(2) + Tsy_fun_positivo(x(1));                             
        ddyw = (uF_y_aplicar - Ty - S*cos(x(5)) - ((K_AIREA*x(6)*sin(x(5)))/x(9)))/(mw + IMOTy);                                
    elseif qy == -1                                                        
        Ty = Tdy_negativo*x(2) -Tsy_fun_negativo(x(1));                               
        ddyw = (uF_y_aplicar -Ty - S*cos(x(5)) - ((K_AIREA*x(6)*sin(x(5)))/x(9)))/(mw + IMOTy);                             
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


    ddb = (1/ (sin(x(5))*x(9))) * (-g*sin(x(7)) -cos(x(7))*dx(4) -2*sin(x(5))*x(10)*x(8) - 2*cos(x(5))*x(9)*x(6)*x(8) )  - 1/(mc*x(9)^2*sin(x(5))^2)*K_AIREB*x(8); % ddBETA

    
     
 
    ddyw_vec(i) = ddyw;
    ddxw_vec(i) = ddxw;
    ddR_vec(i) = ddR;
    
    
   
    ddxc(i) = sin(x(5))*sin(x(7))*ddR + ddxw + 2*cos(x(5))*sin(x(7))*x(10)*x(6) + 2*cos(x(7))*sin(x(5))*x(10)*x(8) - sin(x(5))*sin(x(7))*x(9)*x(6)^2 - sin(x(5))*sin(x(7))*x(9)*x(8)^2 + cos(x(5))*sin(x(7))*x(9)*dda + cos(x(7))*sin(x(5))*x(9)*ddb + 2*cos(x(5))*cos(x(7))*x(9)*x(6)*x(8);
    ddyc(i) = cos(x(5))*ddR + ddyw - cos(x(5))*x(9)*x(6)^2 - sin(x(5))*x(9)*dda - 2*sin(x(5))*x(10)*x(6);
    ddzc(i) = 2*sin(x(5))*sin(x(7))*x(10)*x(8) - 2*cos(x(5))*cos(x(7))*x(10)*x(6) - cos(x(7))*sin(x(5))*ddR + cos(x(7))*sin(x(5))*x(9)*x(6)^2 + cos(x(7))*sin(x(5))*x(9)*x(8)^2 - cos(x(5))*cos(x(7))*x(9)*dda + sin(x(5))*sin(x(7))*x(9)*ddb + 2*cos(x(5))*sin(x(7))*x(9)*x(6)*x(8);



end







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
%               1           2           3           4           5               6   7              8           9            10          11            12           13        14       15  16    17         18  19      20        21   22   

ESTADO_OUT = registro_q_expandido;

end
