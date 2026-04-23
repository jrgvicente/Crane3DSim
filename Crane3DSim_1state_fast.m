%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%       J.VICENTE - j.vicente@unizar.es           LAST UPDATE: 22/04/2026
%       [V2.1]
%       Universidad de Zaragoza - Instituto de Investigacion en Ingenieria
%       de Aragon
%
%  ----------------------------------------------------------------------------
%  ----------------------------------------------------------------------------
% 
% Secondary Function to detect the first state on Crane3Dsim
%
%
%  ------ WHAT'S NEW? ---------------------------------------------------------
%
% V2.0 -- Fixed problems with oscilation friction terms on the X and Y trolley dynamics
% V2.1 -- Fixed bug on friction term on the X dyn ec. and fixed rope
% tension when no rope length change.
%
%
%  ---------------------------------------------------------------------------
%
%% Function inputs:
% Discretization period (somewhat small)
% Previous state X_total_k
% Previous action inputs (in k)
% State of x, y, z (in k)
% Action input at next instant (in k+1)
%
%% Function outputs:
% New states to start simulating at k(+) with the new input of
% action
%
%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

function [qx_new, qy_new, qr_new, X_k1, T_k1] = Crane3DSim_1state_fast(Tdiscret, X_k, uF_x_k1, uF_y_k1 , uF_r_k1 , mw, ms, mc, Tsx_fun_positivo, Tsx_fun_negativo , Tsy_fun_positivo, Tsy_fun_negativo, Tsr_fun_positivo, Tsr_fun_negativo, Tdy_positivo,Tdy_negativo, Tdx_positivo, Tdx_negativo, Tdr_positivo, Tdr_negativo, g, K_AIREA, K_AIREB, xlim_positivo, xlim_negativo, ylim_positivo, ylim_negativo, qx, qy, qr,IMOT, IMOTx, IMOTy)


%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% I CALCULATE THE STATE AT INSTANT K+1 WITH EULER METHOD
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%


x0 = X_k;

%1. I calculate the derivative of the state at the current instant with the same
%function that I used for the ode
dx_dt = func_modo1(0, X_k, uF_x_k1, uF_y_k1, uF_r_k1, mw, ms, mc, Tsx_fun_positivo, Tsx_fun_negativo , Tsy_fun_positivo, Tsy_fun_negativo, Tsr_fun_positivo, Tsr_fun_negativo, Tdy_positivo,Tdy_negativo, Tdx_positivo, Tdx_negativo, Tdr_positivo, Tdr_negativo, g, K_AIREA, K_AIREB, qx, qy, qr, IMOT, IMOTx, IMOTy);

%2. I calculate the approximation of the state at instant k+1 assuming
%derivative cte at discretization time.
X_k1 = X_k + Tdiscret * dx_dt'; 

T_k1 = Tdiscret; 



%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%  I CALCULATE THE NEW STATUSES BASED ON THE PREVIOUS STATUS AND THE PREVIOUS X.
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%




%% 1 - FRICTION CALCULATION

%% X-AXIS
    if qx == 0                                                              % No movement in x
        Tx = 0;     
    elseif qx == 1                                                          % It moves in the +x direction (positive velocity)
        Tx = Tdx_positivo*X_k1(4) + Tsx_fun_positivo(X_k1(3));                              % [N] (absolute value, with direction but no sign)
    elseif qx == -1                                                         % It moves in the -x direction (negative velocity)
        Tx = Tdx_negativo*X_k1(4) - Tsx_fun_negativo(X_k1(3));                              % [N] (absolute value, with direction but no sign)
    end


%% Y-AXIS
    if qy == 0                                                              % No movement in y
        Ty = 0;
    elseif qy == 1                                                          % It moves in the +y direction (positive velocity)
        Ty = Tdy_positivo*X_k1(2) + Tsy_fun_positivo(X_k1(1));                              % [N] (absolute value, with direction but no sign)
    elseif qy == -1                                                         % It moves in the -y direction (negative velocity)
        Ty = Tdy_negativo*X_k1(2) -Tsy_fun_negativo(X_k1(1));                               % [N] (absolute value, with direction but no sign)
    end




if qr == 0                                                       
    S = -(mc*(X_k1(9)^2*X_k1(6)^2*mw^2 + X_k1(9)^2*X_k1(8)^2*mw^2 - uF_y_k1*X_k1(9)*ms*cos(X_k1(5)) - uF_y_k1*X_k1(9)*mw*cos(X_k1(5)) + X_k1(9)^2*X_k1(6)^2*ms*mw ...
                    + X_k1(9)^2*X_k1(8)^2*ms*mw + (K_AIREA*X_k1(6)*ms*sin(2*X_k1(5)))/2 - (K_AIREB*X_k1(8)*mw*sin(2*X_k1(7)))/2 - X_k1(9)^2*X_k1(8)^2*mw^2*cos(X_k1(5))^2 ...
                    - uF_x_k1*X_k1(9)*mw*sin(X_k1(5))*sin(X_k1(7)) + X_k1(9)*g*mw^2*cos(X_k1(7))*sin(X_k1(5)) + Ty*X_k1(9)*X_k1(2)*ms*cos(X_k1(5)) + Ty*X_k1(9)*X_k1(2)*mw*cos(X_k1(5)) ...
                    - X_k1(9)^2*X_k1(8)^2*ms*mw*cos(X_k1(5))^2 + K_AIREA*X_k1(6)*mw*cos(X_k1(5))*cos(X_k1(7))^2*sin(X_k1(5)) + Tx*X_k1(9)*X_k1(4)*mw*sin(X_k1(5))*sin(X_k1(7)) ...
                    + X_k1(9)*g*ms*mw*cos(X_k1(7))*sin(X_k1(5))))/(X_k1(9)*(mc*mw + ms*mw + mw^2 + mc*ms*cos(X_k1(5))^2 - mc*mw*cos(X_k1(7))^2 + mc*mw*cos(X_k1(5))^2*cos(X_k1(7))^2));
elseif qr == 1                                                         
    Tr = Tdr_positivo*X_k1(10) - Tsr_fun_positivo(X_k1(9));            
    S = uF_r_k1 - Tr;
elseif qr == -1                                                        
    Tr = Tdr_negativo*X_k1(10) + Tsr_fun_negativo(X_k1(9));            
    S = uF_r_k1 - Tr;
end 



% x axis

if qx == 0          
    if (((uF_x_k1 - sin(X_k1(5))*sin(X_k1(7))*S + ((K_AIREA*X_k1(6)*cos(X_k1(5))*sin(X_k1(7)))/X_k1(9) + (K_AIREB*X_k1(8)*cos(X_k1(7)))/(X_k1(9)*sin(X_k1(5))))) - Tsx_fun_positivo(X_k1(3))) * (X_k1(3)<xlim_positivo) )> 0
        qx_new = 1;
    elseif (((uF_x_k1 - sin(X_k1(5))*sin(X_k1(7))*S + ((K_AIREA*X_k1(6)*cos(X_k1(5))*sin(X_k1(7)))/X_k1(9) + (K_AIREB*X_k1(8)*cos(X_k1(7)))/(X_k1(9)*sin(X_k1(5)))))  + Tsx_fun_negativo(X_k1(3))) * (X_k1(3)>xlim_negativo) ) < 0
        qx_new = -1;
    else
        qx_new = 0;
    end
else              
    if abs(X_k1(4)) < 1e-4
        qx_new = 0;
    else
        qx_new = qx;
    end
end



% Y-axis

if qy == 0         
    if (((uF_y_k1 -S*cos(X_k1(5)) - ((K_AIREA*X_k1(6)*sin(X_k1(5)))/X_k1(9))) - Tsy_fun_positivo(X_k1(1))) * (X_k1(1)<ylim_positivo) )> 0
        qy_new = 1;
    elseif ( ((uF_y_k1 -S*cos(X_k1(5)) - ((K_AIREA*X_k1(6)*sin(X_k1(5)))/X_k1(9))) + Tsy_fun_negativo(X_k1(1))) * (X_k1(1)>ylim_negativo)) < 0
        qy_new = -1;
    else
        qy_new = 0;
    end
else               
    if abs(X_k1(2)) < 1e-4
        qy_new = 0;
    else
        qy_new = qy;
    end
end




% rope axis



if qx_new == 1                 
    dx4 =  (uF_x_k1 - (Tdx_positivo*X_k1(4) + Tsx_fun_positivo(X_k1(3))) - S*sin(X_k1(5))*sin(X_k1(7)) + ((K_AIREA*X_k1(6)*cos(X_k1(5))*sin(X_k1(7)))/X_k1(9) + (K_AIREB*X_k1(8)*cos(X_k1(7)))/(X_k1(9)*sin(X_k1(5)))))/(ms + mw + IMOTx);                    
elseif qx_new == -1             
    dx4 =  (uF_x_k1 - (Tdx_negativo*X_k1(4) - Tsx_fun_negativo(X_k1(3))) - S*sin(X_k1(5))*sin(X_k1(7)) + ((K_AIREA*X_k1(6)*cos(X_k1(5))*sin(X_k1(7)))/X_k1(9) + (K_AIREB*X_k1(8)*cos(X_k1(7)))/(X_k1(9)*sin(X_k1(5)))))/(ms + mw + IMOTx);                 
else
    dx4 = 0;                    
end 


% -- y --

if qy_new == 1                  
    dx2 = (uF_y_k1 - (Tdy_positivo*X_k1(2) + Tsy_fun_positivo(X_k1(1))) - S*cos(X_k1(5)) - ((K_AIREA*X_k1(6)*sin(X_k1(5)))/X_k1(9)))/(mw + IMOTy);                    
elseif qy_new == -1            
    dx2 = (uF_y_k1 - (Tdy_negativo*X_k1(2) - Tsy_fun_negativo(X_k1(1))) - S*cos(X_k1(5)) - ((K_AIREA*X_k1(6)*sin(X_k1(5)))/X_k1(9)))/(mw + IMOTy);             
     
else
    dx2 = 0;    
end

 % -- rope --

S1 = uF_r_k1;

f_cuerda = (S1 - mc*cos(X_k1(5))*dx2 + mc*X_k1(9)*X_k1(6)^2 + mc*X_k1(9)*X_k1(8)^2 - mc*cos(X_k1(5))^2*X_k1(9)*X_k1(8)^2 - mc*sin(X_k1(5))*sin(X_k1(7))*dx4 + g*mc*cos(X_k1(7))*sin(X_k1(5)) );



if qr == 0         
    if f_cuerda < -Tsr_fun_positivo(X_k1(9))
        qr_new = 1;   
    elseif f_cuerda > Tsr_fun_negativo(X_k1(9))
        qr_new = -1;
    else
        qr_new = 0;
        
    end

else 
    qr_new = qr;
end





%% DYN FUNCTION

function dx_dt = func_modo1(t, x, uF_x_aplicar, uF_y_aplicar, uF_r_aplicar, mw, ms, mc, Tsx_fun_positivo, Tsx_fun_negativo , Tsy_fun_positivo, Tsy_fun_negativo, Tsr_fun_positivo, Tsr_fun_negativo, Tdy_positivo,Tdy_negativo, Tdx_positivo, Tdx_negativo, Tdr_positivo, Tdr_negativo, g, K_AIREA, K_AIREB, qx, qy, qr, IMOT, IMOTx, IMOTy)
 


%% 1 - FRICTION CALCULATION

%% X-AXIS
    if qx == 0                                                              % No movement in x
        Tx = 0;     
    elseif qx == 1                                                          % It moves in the +x direction (positive velocity)
        Tx = Tdx_positivo*x(4) + Tsx_fun_positivo(x(3));                              % [N] (absolute value, with direction but no sign)
    elseif qx == -1                                                         % It moves in the -x direction (negative velocity)
        Tx = Tdx_negativo*x(4) - Tsx_fun_negativo(x(3));                              % [N] (absolute value, with direction but no sign)
    end


%% Y-AXIS
    if qy == 0                                                              % No movement in y
        Ty = 0;
    elseif qy == 1                                                          % It moves in the +y direction (positive velocity)
        Ty = Tdy_positivo*x(2) + Tsy_fun_positivo(x(1));                              % [N] (absolute value, with direction but no sign)
    elseif qy == -1                                                         % It moves in the -y direction (negative velocity)
        Ty = Tdy_negativo*x(2) -Tsy_fun_negativo(x(1));                               % [N] (absolute value, with direction but no sign)
    end


%% 2 - TENSION CALCULATION & ROPE FRICTION

%% ROPE TENSION
    if qr == 0                                                              % No movement along the rope's axis
         S = -(mc*(x(9)^2*x(6)^2*mw^2 + x(9)^2*x(8)^2*mw^2 - uF_y_aplicar*x(9)*ms*cos(x(5)) - uF_y_aplicar*x(9)*mw*cos(x(5)) + x(9)^2*x(6)^2*ms*mw ...
                + x(9)^2*x(8)^2*ms*mw + (K_AIREA*x(6)*ms*sin(2*x(5)))/2 - (K_AIREB*x(8)*mw*sin(2*x(7)))/2 - x(9)^2*x(8)^2*mw^2*cos(x(5))^2 ...
                - uF_x_aplicar*x(9)*mw*sin(x(5))*sin(x(7)) + x(9)*g*mw^2*cos(x(7))*sin(x(5)) + Ty*x(9)*x(2)*ms*cos(x(5)) + Ty*x(9)*x(2)*mw*cos(x(5)) ...
                - x(9)^2*x(8)^2*ms*mw*cos(x(5))^2 + K_AIREA*x(6)*mw*cos(x(5))*cos(x(7))^2*sin(x(5)) + Tx*x(9)*x(4)*mw*sin(x(5))*sin(x(7)) ...
                + x(9)*g*ms*mw*cos(x(7))*sin(x(5))))/(x(9)*(mc*mw + ms*mw + mw^2 + mc*ms*cos(x(5))^2 - mc*mw*cos(x(7))^2 + mc*mw*cos(x(5))^2*cos(x(7))^2));              
    elseif qr == 1                                                          % Rope moves upwards                                                         
        Tr = Tdr_positivo*x(10) - Tsr_fun_positivo(x(9));                   % upwards, x(10)<0 and the total force must be <0    
        S = uF_r_aplicar-Tr;       
    elseif qr == -1                                                         % It moves downwards (negative velocity)        
        Tr = Tdr_negativo*x(10) + Tsr_fun_negativo(x(9));                   % downwards, x(10)>0 and the total force must be >0    
        S = uF_r_aplicar-Tr;       
    end     


%% 3 - DYNAMIC ECUATIONS   
    
%% X-AXIS
    if qx == 0                                                              % No movement in x        
        dx(3) = 0;                                                          % dxw    
        dx(4) = 0;                                                          % ddxw        
    else                                                                    % It moves in the +x/-x direction (positive velocity/negative velocity)
        dx(3) =  x(4);                                                      % dxw         
        dx(4) =  (uF_x_aplicar -Tx - S*sin(x(5))*sin(x(7)) + ((K_AIREA*x(6)*cos(x(5))*sin(x(7)))/x(9) + (K_AIREB*x(8)*cos(x(7)))/(x(9)*sin(x(5))))  ) /  (ms + mw + IMOTx);                     % ddxw  <<< --- CAMBIO V2.1 !!!  --->>>
    end

%% Y-AXIS
    if qy == 0                                                              % No movement in y    
        dx(1) = 0;                                                          % dyw    
        dx(2) = 0;                                                          % ddyw        
    else                                                                    % It moves in the +y/-y direction (positive velocity/negative velocity)       
        dx(1) = x(2);                                                       % dyw       
        dx(2) = (uF_y_aplicar - Ty - S*cos(x(5)) - ((K_AIREA*x(6)*sin(x(5)))/x(9))  )/(mw + IMOTy);                            % ddyw    
    end

%% ROPE MOVEMENT
    if qr == 0                                                              % No movement along the rope's axis        
        dx(9) = 0;                                                          % dR        
        dx(10) = 0;                                                         % ddR
    else                                                                    % It moves upwards / downwards      
        dx(9) = x(10);         
        dx(10) = (1/(mc+IMOT)) * (S - mc*cos(x(5))*dx(2) + mc*x(9)*x(6)^2 + mc*x(9)*x(8)^2 - mc*cos(x(5))^2*x(9)*x(8)^2 - mc*sin(x(5))*sin(x(7))*dx(4) + g*mc*cos(x(7))*sin(x(5)) );   
    end



dx(5) = x(6);
dx(7) = x(8);

dx(6) = (1/x(9)) * (-2*x(10)*x(6) + sin(x(5))*dx(2) - cos(x(5))*sin(x(7))*dx(4) + g*cos(x(5))*cos(x(7)) + cos(x(5))*sin(x(5))*x(9)*x(8)^2  ) - 1/(mc*x(9)^2)*K_AIREA*x(6);     % ddALFA

dx(8) = (1/ (sin(x(5))*x(9))) * (-g*sin(x(7)) -cos(x(7))*dx(4) -2*sin(x(5))*x(10)*x(8) - 2*cos(x(5))*x(9)*x(6)*x(8) )  - 1/(mc*x(9)^2*sin(x(5))^2)*K_AIREB*x(8);               % ddBETA


dx_dt = transpose(dx);

end


end
