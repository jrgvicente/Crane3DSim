%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%       J.VICENTE - j.vicente@unizar.es           LAST UPDATE: 10/07/2025
%       [V1.0]
%       Universidad de Zaragoza - Instituto de Investigacion en Ingenieria
%       de Aragon
%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%   Crane3DSim_cont Launcher -- ODE Simulation with continuos input signal      %
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

close all
clear
clc


% -----------------------------------------------
%% Physical characteristics of the simulation


mc = 0.457;  % payload mass [Kg]
mw = 1.155;  % trolley mass [Kg] 
ms = 2.20;   % rail mass [Kg]
g = 9.81;    % gravity [m/s2]

masses = [mc mw ms];


xlim_positive = 10000+0.505;    %-- maximum X-axis coordinate of the trolley
xlim_negative = 0-10000;        %-- minimum X-axis coordinate of the trolley
ylim_positive = 10000+0.625;    %-- maximum Y-axis coordinate of the trolley
ylim_negative = 0-10000;        %-- minimum Y-axis coordinate of the trolley
rlim_positive = 1;%100+0.57;    %-- maximum rope-axis coordinate of the trolley
rlim_negative = 0.13;           %-- minimum rope-axis coordinate of the trolley

ph_limits = [xlim_positive xlim_negative ylim_positive ylim_negative rlim_positive rlim_negative]; 


% -----------------------------------------------
%% Initial conditions

tfin = 5;   % Total time of the simulation [seconds]


x0 = zeros(10,1);
x0(1) = 0;                 % initial trolley position on y axis [m]
x0(2) = 0;                 % initial trolley velocity on y axis [m]
x0(3) = 0.1;                 % initial trolley position on x axis [m]
x0(4) = 0.0;               % initial trolley velocity on x axis [m]
x0(5) = pi/2;              % initial alpha angle [rad]
x0(6) = 0;                 % initial alpha anglular velocity [rad/s]
x0(7) = 0;                 % initial beta angle [rad]
x0(8) = 0;                 % initial beta anglular velocity [rad/s] 
x0(9) = 0.5;               % initial cable length [m]
x0(10) = 0;                % initial cale velocity [m/s]
 



% -----------------------------------------------
%% Input force signal functions @(t) :: PWM [-1 - 1] ::


% Example

amplitud = 1;
bias = 0;
frecuencia_rad_seg = 1.50;
fase_rad = 0;

uPWM_x  = @(t) 0.8*amplitud * sin(frecuencia_rad_seg * t + fase_rad) + bias; % Input PWM on x-axis motor
uPWM_y = @(t) 0; % Input PWM on y-axis motor
uPWM_z = @(t) 0; % Input PWM on rope-axis motor



% -----------------------------------------------
%% Load the identification .mat
% see the ParametersMATgenerator generator code

load('Example_identification.mat');
%Crane3DSim_IdentifParameters



% -----------------------------------------------
%% variable type changes


ks = [X_PWM_TO_F Y_PWM_TO_F Z_PWM_TO_F];

fricciones = struct();

fricciones.Tsx_positive =  @(x) fsecaXPOSITIVO_fnc(x); 
fricciones.Tsx_negative =  @(x) fsecaXNEGATIVO_fnc(x); 

fricciones.Tsy_positive = @(y) fsecaYPOSITIVO_fnc(y); 
fricciones.Tsy_negative = @(y) fsecaYNEGATIVO_fnc(y); 

fricciones.Tsr_positive =  @(l) Tsr_positivo/Z_PWM_TO_F; 
fricciones.Tsr_negative = @(l) Tsr_negativo/Z_PWM_TO_F; 

fricciones.Tdy_positive = Tdy_positivo/Y_PWM_TO_F;   
fricciones.Tdy_negative = Tdy_negativo/Y_PWM_TO_F;   

fricciones.Tdx_positive = Tdx_positivo/X_PWM_TO_F;  
fricciones.Tdx_negative = Tdx_negativo/X_PWM_TO_F;  

fricciones.Tdr_positive = Tdr_positivo/Z_PWM_TO_F;     
fricciones.Tdr_negative = Tdr_negativo/Z_PWM_TO_F; 

fricciones.K_AIREB = K_AIREB;
fricciones.K_AIREA = K_AIREA;



% -----------------------------------------------
%% SIMULATION

[T_out,DATA_OUT,ESTADO_OUT] = Crane3DSim_cont(fricciones,IMOTr,IMOTx,IMOTy,ks,uPWM_x,uPWM_y,uPWM_z,masses,x0,tfin,g,ph_limits);



