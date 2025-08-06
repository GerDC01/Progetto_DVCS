%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%               Dinamica, Controllo di Veicoli Spaziali
%                           A.a. 2020-2021
%
%           Progetto Gruppo 11 - Qsat-EOS

% ATTENSIONE, RUNNARE CON MATLAB POST R2016b ( Introduzione Arithmetic Expansion)

clear
clc
close all

%% Section 0A - Earth Constants
mu              = 3.986012*10^14;                                          % [m^3/s^2]  Standard gravitational parameter
g0              = 9.81;                                                    % [m/s^2]    Zero gravity
r_E             = 6378.145*10^3;                                           % [m]        Earth Radius

%% Section 0B - Simulation Setup
tsp_0           = 0.02;                                                    % [s]   Zero shoot time
Dt              = 0.01;                                                    % [s]   Time step of simulation
Dt_guid         = 1;                                                       % [s]   Update sample time of the guidance algorithm
Dt_con          = 0.02;                                                    % [s]   Update sample time of the control algorithm
tfin            = 10000;                                                   % [s]   Maximum time of simulation (!can be modified)
cont            = 0;                                                       % [s]   Index
t               = 0:Dt:tfin;                                               % [s]   Vector of time

%% Section 0C - Target Informations
height_target   = 400*10^3;                                                % [m]        Reference altitude
r_target        = r_E + height_target;                                     % [m]        Orbital Radius
omega_target    = (mu/r_target^3)^0.5;                                     % [rad/s]      Target angular velocity
v_target        = sqrt(mu/r_target);                                       % [m/s]      Tangential orbital speed
T_target        = 2*pi/omega_target;                                       % [s]

%% Section 1 - Chaser Informations

% Orbit ------------------------------------------------------------------
height_chaser  = 397*10^3;                                                 % [m]    Altitude Chaser
r_chaser       = r_E + height_chaser;                                      % [m]    Orbit radius Chaser
omega_chaser   = sqrt(mu/r_chaser^3);                                      % [rad/s]  Chaser Angular Velocity

% Inertia ----------------------------------------------------------------
m_c0            = 50;                                                      % [kg] initial chaser mass  
lx              = 0.49;                     
ly              = 0.50;                     
lz              = 0.50;                  
Jx0             =(m_c0*(2*lx^2)/12);          
Jy0             =(m_c0*(2*ly^2)/12);       
Jz0             =(m_c0*(2*lz^2)/12);

J0              =[Jx0    0    0;                 
                    0  Jy0    0;               
                    0    0  Jz0];                                          % [kg*m^2]  Inertial tensor

% Thrusters --------------------------------------------------------------
Isp1            = 220;                                                     % [s]     Specific impulse
c1              = Isp1*g0;                                                 % [m/s]   Dumping speed
Tmax            = 1;                                                       % [N]     Maximum thrust
n               = 2;                                                       % [--]    Number of thrusters (1X+3X) or (3Z+4Z)

Total_max_thrust= n*Tmax;                                                  % [N]    Total thrust at max output both thrusters

% Reaction Wheels ---------------------------------------------------------
RW_max_torque   = 5e-3;                                                    % [N*m]       RW Maximum torque
Is              = 0.02;                                                    % [kg*m^2]    RW Moment of inertia
om_0            = [0,0,0]';
tau_RW          = 1;                                                       % [s]         RW Time constant
p_RW            = 0.6;                                                     % [rad/s]     RW Pole
Kr              = 1;

%% Section 2 - External Disturbance
% THE USER CAN CONSIDER THESE ERRORS IN COMBINATION WITH THE DETAILED MODEL
% OF GRAVITY GRADIENT AND J2 EFFECT.

% Constant and random errors are considered for the forces.
% Constant error is considered for the moments.
%--------------------------------------------------------------------------
%   Thrust errors due to external disturbances
%--------------------------------------------------------------------------

F_err           = [0.01,0.01,0.01];                                        % [N]  Thrust constant error
F_err_min     	= [0,0,0];                                                 % [N]  Error mean value
F_err_var       = [0.00001,0.00001,0.00001];                               % [N]  Variance of the random error 

Sample_time_F 	= 1  ;                                                      % [s]  Sampling time for the thrust error
                                                                           
%--------------------------------------------------------------------------
%   Constant error for the moment due to external disturbances
%--------------------------------------------------------------------------

M_ext           = [0.01,0.01,0.01];                                        % [N*m]  Moment constant error
%--------------------------------------------------------------------------

%--------------------------------------------------------------------------
%   Thrust due to air drag model
%--------------------------------------------------------------------------

rho             = 1*10^(-12);                                              % [kg/m^3] Air density obtained as medium value for 500 km orbit Fehse
Vx              = omega_target*r_target;                                   % [m/s]    Orbital velocity
CD              = 2.2;                                                     % [--]     Drag coefficient for satellite of compact shapes
A               = ly*lz;                                                   % [m^2]    Cross section
F_D             = 1/2*rho*CD*A*Vx^2;                                       % [N]      Force due to the drag

% Data for simulation
% Attitude angles as constant vector -- TO BE DELETED if the attitude control is included.
att_const       = [0.01;0.01;0.01]';


%% Section 3 - Hohmann

% Initial conditions ------------------------------------------------------

% ALL POSITIONS ASSIGNED IN LVLH FRAME

x0              = -10*10^3;                                                % [m]      X position 
%%%% WARNING on X position: this value has to be modify if a CONTINUOUS maneuver is implemented!!!!
y0              = 0;                                                       % [m]      Y position

zc              = height_chaser;                                           % [m]   Chaser orbit
zt              = height_target;                                           % [m]   Target orbit
delta_z         = zt - zc;                                                 % [m/s] Difference of orbit between Chaser and Target 
z0              = delta_z;

Vx0_Free_Drift  = 3/2*omega_target*delta_z;                                % [m/s] Relative speed in LVLH when the Target and Chaser orbit is different
% 
%
% 
x0_dot          = Vx0_Free_Drift;                                          % [m/s]    X speed 
y0_dot          = 0;                                                       % [m/s]    Y speed 
z0_dot          = 0;                                                       % [m/s]    Z speed

omega_0         = omega_target;
omega0          = [0,-omega_0,0];                                          % [rad/s]  Chaser angular velocity in the 
%                                                                          %          local orbital frame
eul_ang0        = [0,0,0];                                                 % [rad]    Euler angles between the orbital 
%                                                                          %          reference frame and the local orbital frame
%                                                                          %          (chaser) -- only if aligned
Pos0            = [x0 y0 z0]';
Vel0            = [x0_dot y0_dot z0_dot]';

%% Section 3A - Ideal Maneuver --------------------------------------------

deltaVx         = omega_chaser/4*Pos0(3);
deltax          = 3*pi/omega_chaser*deltaVx;
deltax_FD       = 0;

T_1fire         = abs(deltax_FD)/x0_dot;                                   % [s] Istante primo sparo Hohmann
a_Hohmann       = (r_chaser +r_target )/2;
T_Hohmann       = 2*pi*sqrt(a_Hohmann^3/mu);
T_2fire         = T_1fire+ T_Hohmann/2;                                    % [s] Istante secondo sparo Hohmann

% T_end_Hohmann = T_2fire + 1000;                                          % [s] Istante di Fine Simulazione Hohmann
T_end_Hohmann = 2*T_Hohmann; % Usare questo se manovra singola

% Open-Loop Maneuver
% ATTENTION, these values will be rewritten for the subsequent maneuvers
deltat_onOP    = m_c0*deltaVx/Total_max_thrust;                            % Tempo di accensione Thruster per la manovra di Hohmann
lambda         = 0.5;                                                      % Per SMC
S              = 0.9;                                                      % Per SMC

tau            = 1.2;
Uon            = 1.25;
Uoff           = 0.55;
deltat_on      = 0.025;
KF             = (Uon-Uoff)/(1-exp(-deltat_on/tau));

%% Section 3B SIMULATION --------------------------------------------------
Hohmann_out=sim("Hohmann.slx");

% Fornisce in output:

% Hohmann_out.Positions_IM                                                 % [m] Posizioni xyz in LVLH manovra ideale
% Hohmann_out.Vbar_IM                                                      % [m] X LVLH manovra ideale (1x1)
% Hohmann_out.Rbar_IM                                                      % [m] Z LVLH manovra ideale (1x1)
% Hohmann_out.Velocity_IM                                                  % [m/s] Velocità LVLH (Vettore 3x1)
% Hohmann_out.F_OL                                                         % [N] Forze applicate dai thuster nella manovra in open loop (3x1)
% Hohmann_out.Hohmann_Position_CLose_Loop                                  % [m] Posizioni manovra in close loop
% Hohmann_out.Hohmann_Velocity_Close_Loop                                  % [m/s] Velocità manovra in close loop
% Hohmann_out.Hohmann_Position_Open_Loop                                   % [m] Posizioni manovra in Open loop
% Hohmann_out.Hohmann_Velocity_Open_Loop                                   % [m/s] Velocità manovra in Open loop
 

%% Section 3C RESULTS PLOT ------------------------------------------------

Vbar_IM         = Hohmann_out.Vbar_IM;                                     % [m] X LVLH
Rbar_IM         = Hohmann_out.Rbar_IM;                                     % [m] Z LVLH
Velocity_IM     = Hohmann_out.Velocity_IM;                                 % [m/s]
F_OL            = Hohmann_out.F_OL;

figure(1)
sgtitle('Manovra di Hohmann')

subplot(2,2,1)
plot(Vbar_IM.data, Rbar_IM.data)                                           % Plotta x e z LVLH dalle eq di Hill
set(gca, 'YDir', 'reverse');
set(gca, 'XDir', 'reverse');

xlabel('X LVLH (Vbar)')
ylabel('Z LVLH (Rbar)')
title('Ideal Maneuver Positions')
grid on

subplot(2,2,2)
plot(Velocity_IM.time, Velocity_IM.data)                                   % Plotta Le velocità in funzione del tempo
xlabel('Time')
ylabel('Velocity')
legend('V\_Vbar', 'V\_Hbar', 'V\_Rbar')
title('Ideal Maneuver Velocity')
grid on


subplot(2,2,[3,4])
plot(F_OL.time, F_OL.data)                                                 % Plotta le attivazioni del thruster in funzione del tempo
xlabel('time')
ylabel('Thrust')
title('Spari')
grid on


%% Section 4 - Radial Boost

% Initial conditions ------------------------------------------------------

Vx0_RB          = v_target;                                                % [m/s]     Initial speed (only for study in inertial frame)
 
x0_RB           = -3*10^3;                                                 % [m]      X position 
y0_RB           = 0;                                                       % [m]      Y position 
z0_RB           = 0;                                                       % [m]      Z position

x0_dot_RB       = Vx0_RB;                                                  % [m/s]    X speed 
y0_dot_RB       = 0;                                                       % [m/s]    Y speed 
z0_dot_RB       = 0;                                                       % [m/s]    Z speed
omega0          = [0,0,0];                                                 % [rad/s]  Target angular velocity in the 
                                                                           %          local orbital frame

eul_ang0        = [0,0,0];                                                 % [rad]    Euler angles between the orbital 
                                                                           %          reference frame and the local orbital frame
                                                                           
                                                                           %          (chaser) -- only if aligned
Pos0_RB         = [x0_RB y0_RB z0_RB]';
Vel0_RB         = [x0_dot_RB y0_dot_RB z0_dot_RB]';

% Student Edited Part -----------------------------------------------------

pf_HOH          = Pos0_RB;                                                 % Posizione termine Hohmann
p0_id_RB        = [-3000 0 0];                                             % Posizione teorica inizio manovra
p0_RB           = p0_id_RB;                                                % Posizione effettiva di inizio manovra
deltaz_RB = pf_HOH(3) - p0_RB(3);                                          % [m] Differenza quota RB
dp0_RB = [3/2*omega_target*deltaz_RB 0 0];                                 % [m/s] Velocità inizion RB 

pf_id_RB        = [-500 0 0];                                              % Posizione di termine manovra RB

T1_RB           = 10;                                                      % [s] Istante Primo Sparo !Grandezza non nulla solo per evidenziare momento di inizio sparo

T_RB            = (2*pi)/omega_target;                                     % [s] Periodo orbita di trasferimento  
T2_RB           = T1_RB + T_RB/2;                                          % [s] Istante del secondo sparo

% T_end_RB = T2_RB + 1000; %Usare questo se manovra completa                 % [s] Istante di Fine Simulazione Hohmann
T_end_RB = 2*T_RB; % Usare questo se manovra singola

delta_x_RB      = pf_id_RB(1) - p0_RB(1);                                  % [m] Variazione di posizione imposta dalla manovra
delta_Vz_RB     = (omega_target/4)*delta_x_RB;                             % [m/s] Variazione istantanea di velocità a seguito dello sparo

% Definizione Costanti Guida ----------------------------------------------

% SMC
landa_RB        = 1;
S_RB            = 0.5;

%PWPF
Uon             = 1;                                                       % Imposto
Uoff            = 0.7;                                                     % Trovato per tentativi
kf              = 4;                                                       % Valore dettato dalla relazione con K_on e DB
ton             = 0.2;                                                     % [s] Imposto tenendo conto della necessità di incrementarne la durata per ridurre gli spari pur tenendo basso il valore (ipotesi di sparo impulsivo)
tau             = -ton/(log(1-(Uon-Uoff)/(Tmax*kf)));

%% Section 4B SIMULATION --------------------------------------------------
Radial_Boost = sim('RB.slx');

% Fornisce in output:

% out.p_des_Rb                                                             % [m] Posizione desiderata blocco guida Radial Boost
% out.dp_des_Rb                                                            % [m/s] Velocità desiderata blocco guida Radial Boost
% out.p_diff_RB                                                            % [m] Errore Posizione blocco SMC Radial Boost
% out.dp_diff_RB                                                           % [m/s] Errore Velocità blocco SMC Radial Boost
% out.ctrl_RB_x                                                            % [N] Thrust di Controllo su X RB
% out.ctrl_RB_y                                                            % [N] Thrust di Controllo su Y RB
% out.ctrl_RB_z                                                            % [N] Thrust di Controllo su Z RB
% out.F_RB_X                                                               % [N] Thrust effettiva su X RB
% out.F_RB_Y                                                               % [N] Thrust effettiva su Y RB
% out.F_RB_Z                                                               % [N] Thrust effettiva su Z RB
% out.p_RB                                                                 % [m] Posizione effettiva Radial Boost
% out.dp_RB                                                                % [m/s] Velocità effettiva Radial Boost

%% Section 4C RESULTS PLOT ------------------------------------------------

% Traiettoria
p_id_RB         = Radial_Boost.p_des_RB;
dp_id_RB        = Radial_Boost.dp_des_RB;
p_RB            = Radial_Boost.p_RB;
dp_RB           = Radial_Boost.dp_RB;

[px_id_RB, py_id_RB, pz_id_RB]      = calcolo_componenti(p_id_RB);         % Posizioni ideali
[dpx_id_RB, dpy_id_RB, dpz_id_RB]   = calcolo_componenti(dp_id_RB);        % Velocità ideali
[px_RB, py_RB, pz_RB]               = calcolo_componenti(p_RB);            % Posizioni effettive
[dpx_RB, dpy_RB, dpz_RB]            = calcolo_componenti(dp_RB);           % Velocità effettive

% Errori 
p_diff_RB       = Radial_Boost.p_diff_RB;
dp_diff_RB      = Radial_Boost.dp_diff_RB;
% Azioni di controllo
ux              = Radial_Boost.ctrl_RB_x;
uy              = Radial_Boost.ctrl_RB_y;
uz              = Radial_Boost.ctrl_RB_z;

% Forza di controllo
Fx              = Radial_Boost.F_RB_x;
Fy              = Radial_Boost.F_RB_y;
Fz              = Radial_Boost.F_RB_z;

% Posizione
figure
plot(px_id_RB,pz_id_RB, '--b', 'LineWidth', 2)
hold on
plot(px_RB, pz_RB, 'LineWidth', 2)
hold on
set(gca, 'XDir', 'reverse');  % Inverte l'asse X
set(gca, 'YDir', 'reverse');  % Inverte l'asse Y
legend('Ideal Trajectory', 'Real Trajectory')
title("Manouvre RB",'FontSize',12)
xlabel("X-bar [m]")
ylabel("Z-bar [m]")
grid on
hold off

% Velocità
figure
plot(dp_RB.Time, dp_RB.Data, 'LineWidth', 2)
hold on 
plot(dp_id_RB.Time, dpx_id_RB, '--k')
hold on 
plot(dp_id_RB.Time, dpy_id_RB, '--k')
hold on 
plot(dp_id_RB.Time, dpz_id_RB, '--k')
hold on
legend('vx', 'vy', 'vz', 'vx\_id', 'vy\_id', 'vz\_id')
title("Vettore velocità RB",'FontSize',14)
xlabel("time [s]")
ylabel("Speed [m/s]")
grid on
hold off

% Errore posizione
figure
plot(p_diff_RB.Time, p_diff_RB.Data, 'LineWidth', 2)
hold on
legend('x', 'y', 'z')
title("Position Error RB",'FontSize',12)
xlabel("Time [s]")
ylabel("Error [m]")
grid on
hold off

% Errore Posizione
figure
plot(dp_diff_RB.Time, dp_diff_RB.Data, 'LineWidth', 2)
hold on
legend('vx', 'vy', 'vz')
title("Speed Error RB",'FontSize',12)
xlabel("Time [s]")
ylabel("Error [m]")
grid on
hold off

f2 = figure('Name', "Azioni di controllo"); 
hold on
subplot(3,1,1)
plot(ux.Time, ux.Data, 'LineWidth', 2)
hold on
plot(Fx.Time, Fx.Data, 'LineWidth', 2)
legend('Control Force', 'Applied Force')
xlabel("time [s]")
ylabel("X")
grid on

subplot(3,1,2)
plot(uy.Time, uy.Data, 'LineWidth', 2)
hold on
plot(Fy.Time, Fy.Data, 'LineWidth', 2)
legend('Control Force', 'Applied Force')
xlabel("time [s]")
ylabel("Y")
grid on

subplot(3,1,3)
plot(uz.Time, uz.Data, 'LineWidth', 2)
hold on
plot(Fz.Time, Fz.Data, 'LineWidth', 2)
legend('Control Force', 'Applied Force')
xlabel("time [s]")
ylabel("Z")
grid on
hold off



%% Section 5 - Cone Approach

% Initial conditions ------------------------------------------------------

Vmin            = 0.01;                                                    % [m/s] Minimum speed of thruster, Vx slide 35 Progetto
Vx0             = 0.15;                                                    % [m/s] Initial speed - first impulse slide 35 Progetto
x0_CA           = -500;                                                    % [m]      X position 
y0_CA           = 0;                                                       % [m]      Y position 
z0_CA           = 0;                                                       % [m]      Z position
x0_dot_CA       = Vx0;                                                     % [m/s]    X speed 
y0_dot_CA       = 0;                                                       % [m/s]    Y speed 
z0_dot_CA       = 0;                                                       % [m/s]    Z speed
omega0          = [0,0,0];                                                 % [rad/s]  Chaser angular velocity in the 
                                                                           %          local orbital frame
%                                                           
eul_ang0        = [0,0,0];                                                 % [rad]    Euler angles between the orbital 
                                                                           %          reference frame and the local orbital frame
                                                                           %          (chaser) - only if aligned
Pos0_CA         = [x0_CA  y0_CA  z0_CA ]';
Vel0_CA         = [x0_dot_CA  y0_dot_CA  z0_dot_CA ]';

%--------------------------------------------------------------------------
% Final point
%--------------------------------------------------------------------------
xf              = -2;                                                      % [m]  
yf              = 0;                                                       % [m]
zf              = 0;                                                       % [m]

%--------------------------------------------------------------------------    
% Cone section elements
%--------------------------------------------------------------------------    
d1              = 500;                                                     % [m]   distance between target and chaser
r1              = 1;                                                       % [m]   at point 1 (initial point) maximum height
r2              = 0.1;                                                     % [m]   at point 2 (final point) maximum height
teta            = atan((r1 - r2)/d1);                                      % [rad] inclination angle

% Student Edited Part -----------------------------------------------------

% Valori iniziali

p0_CA           = Pos0_CA;                                                 % [m] Posizione
delta_z_CA      = 0;
dp0_CA          = [3/2*omega_target*delta_z_CA 0 0];                       % [m/s] Velocità

% Valori FInali
pf_id           = [-4 0 0];                                                % [m] Posizione Finale ideale 
% FLAG, a riga 416 la capello mette -2, qui è -4
dpf_id          = [0.05 0 0];                                              % [m/s] Vocistà finale ideale

% spari guida 
deltax_CA       = pf_id(1) - p0_CA(1);
deltaVx_CA      = 0.15;
T1_CA           = 10;
T2_CA           = T1_CA + deltax_CA/deltaVx_CA;

% Valori per la guida

% APF
dx_max          = 0.1;
Ka              = 1;
% SMC (APF)
landa_CA_APF    = 0.01;
S_CA_APF        = 0.01;

% SMC guida
landa_CA        = 1.001;
S_CA            = 0.35;

% PWPF
db              = 0.25;
Uon             = 1;
Uoff            = 0.7;
kf              = 4;
ton             = 0.2;
tau             = -ton/(log(1-(Uon-Uoff)/(Tmax*kf)));

%% Section 5B SIMULATION --------------------------------------------------
Cone_approach   = sim("Cone_approach.slx");

% Fornisce in output:

% out.p_Des_CA
% out.dp_des_CA
% out.p_diff_CA                                                            % [m] Errore Posizione blocco SMC Cone Approach
% out.dp_diff_CA                                                           % [m] Errore Velocità blocco SMC Cone Approach
% out.ctrl_CA_x                                                            % Forza teorica di controllo trhuster Cone Approach (SMC)
% out.ctrl_CA_y
% out.ctrl_CA_z
% out.F_CA_x                                                               % Forza effettiva trhuster Cone Approach (PWPF)
% out.F_CA_y
% out.F_CA_z
% out.px                                                                   % Posizione effettiva Cone Apporach (Blocco Hill)
% out.py      
% out.pz
% out.p_CA                                                                 % Posizione effettiva Cone Apporach (Blocco Hill)
% ouyt.dp_CA                                                               % Velocità effettiva Cone Apporach (Blocco Hill)

%% Section 5C RESULTS PLOT -------------------------------------------------
% traiettorie
p_id_CA         = Cone_approach.p_des_CA;                                  % posizione ideale cone approach
dp_id_CA        = Cone_approach.dp_des_CA;                                 % velocità ideale cone approach
p_CA            = Cone_approach.p_CA;                                      % posizione reale cone approach
dp_CA           = Cone_approach.dp_CA;                                     % velocità reale cone approach

[px_id_CA, py_id_CA, pz_id_CA]      = calcolo_componenti(p_id_CA);
[dpx_id_CA, dpy_id_CA, dpz_id_CA]   = calcolo_componenti(dp_id_CA);
[px_CA, py_CA, pz_CA]               = calcolo_componenti(p_CA);
[dpx_CA, dpy_CA, dpz_CA]            = calcolo_componenti(dp_CA);

% Errori
p_diff_CA       = Cone_approach.p_diff_CA;
dp_diff_CA      = Cone_approach.dp_diff_CA;
% Controllo
ux              = Cone_approach.ctrl_CA_x;
uy              = Cone_approach.ctrl_CA_y;
uz              = Cone_approach.ctrl_CA_z;

% Forze
Fx              = Cone_approach.F_CA_x;
Fy              = Cone_approach.F_CA_y;
Fz              = Cone_approach.F_CA_z;

% Curve conica
x_i             = -500;                                                    % [m] Posizione iniziale manovra CA
r_i             = 1;                                                       % [m] Raggio conica iniziale 
x_f             = -4;                                                      % [m] Posizione finale manovra CA
r_f             = 0.1;                                                     % [m] Raggio conica finale
phi_conica      = abs(atan((r_i - r_f)/x_i));                              % [rad] Semiapertura della conica 
phi_conica_deg  = phi_conica*180/pi; 
x_line          = linspace(x_i, x_f, 100);                                 % 100 punti tra x_start e x_end
z_line1         = linspace(r_i, r_f, 100);                                 % Linea superiore
z_line2         = linspace(-r_i, -r_f, 100);                               % Linea inferiore

% posizione CA
figure
plot(px_id_CA,pz_id_CA, 'LineWidth', 2)
hold on 
plot(px_CA,pz_CA, 'LineWidth', 2)
hold on
plot(x_line, z_line1, '--k', 'LineWidth', 2)                               % Linea superiore
hold on
plot(x_line, z_line2, '--k', 'LineWidth', 2)                               % Linea inferiore
hold on
set(gca, 'XDir', 'reverse');                                               % Inverte l'asse X
set(gca, 'YDir', 'reverse');                                               % Inverte l'asse Y
legend('Ideal trajectory', 'Real Trajectory')
title("Manouvre Cone Approach",'FontSize',12)
xlabel("X-bar [m]")
ylabel("Z-bar [m]")
grid on
hold off

% velocità CA
figure
plot(dp_id_CA.Time, dpx_id_CA, '--k')
hold on
plot(dp_id_CA.Time, dpy_id_CA, '--k')
hold on
plot(dp_id_CA.Time, dpz_id_CA, '--k')
hold on
plot(dp_CA.Time, dp_CA.Data, 'LineWidth', 2)
hold on
legend('vx\_id', 'vy\_id', 'vz\_id', 'vx', 'vy', 'vz')
title('Speed Component Cone Approach', 'FontSize', 12)
xlabel("time [s]")
ylabel("Speed [m/s]")
grid on
hold off

% Errore posizione
figure
plot(p_diff_CA.Time, p_diff_CA.Data, 'LineWidth',2)
hold on
legend('x', 'y', 'z')
title("Position Error Cone Approach",'FontSize',12)
xlabel("Time [s]")
ylabel("Position Error [m]")
grid on
hold off

% Errore velocità
figure
plot(dp_diff_CA.Time, dp_diff_CA.Data, 'LineWidth',2)
hold on
legend('vx', 'vy', 'vz')
title("Speed Error Cone Approach",'FontSize',12)
xlabel("Time [s]")
ylabel("Speed Error [m/s]")
grid on
hold off

f2 = figure('Name', "Control Action"); 
hold on
subplot(3,1,1)
plot(ux.Time, ux.Data, 'LineWidth', 1.5)
hold on
plot(Fx.Time, Fx.Data, 'LineWidth', 1.5)
legend('Control Force', 'Applied Force')
xlabel("time [s]")
ylabel("X")
grid on

subplot(3,1,2)
plot(uy.Time, uy.Data, 'LineWidth', 1.5)
hold on
plot(Fy.Time, Fy.Data, 'LineWidth', 1.5)
legend('Control Force', 'Applied Force')
xlabel("time [s]")
ylabel("Y")
grid on

subplot(3,1,3)
plot(uz.Time, uz.Data, 'LineWidth', 1.5)
hold on
plot(Fz.Time, Fz.Data, 'LineWidth', 1.5)
legend('Control Force', 'Applied Force')
xlabel("time [s]")
ylabel("Y")
grid on
hold off


%% Section 6 - Complete Maneuvre ------------------------------------------

% RUN UP TO SECTION 3 (EXCLUDED) BEFORE RUNNING THIS

% Initial conditions ------------------------------------------------------
% Coincident with Hohmann Initial Condition on Section 3 & 3A, except for end time

% ALL POSITIONS ASSIGNED IN LVLH FRAME

x0              = -10*10^3;                                                % [m]      X position 
y0              = 0;                                                       % [m]      Y position

zc              = height_chaser;                                           % [m]   Chaser orbit
zt              = height_target;                                           % [m]   Target orbit
delta_z         = zt - zc;                                                 % [m/s] Difference of orbit between Chaser and Target 
z0              = delta_z;

Vx0_Free_Drift  = 3/2*omega_target*delta_z;                                % [m/s] Relative speed in LVLH when the Target and Chaser orbit is different
% 
%
% 
x0_dot          = Vx0_Free_Drift;                                          % [m/s]    X speed 
y0_dot          = 0;                                                       % [m/s]    Y speed 
z0_dot          = 0;                                                       % [m/s]    Z speed

omega_0         = omega_target;
omega0          = [0,-omega_0,0];                                          % [rad/s]  Chaser angular velocity in the 
%                                                                          %          local orbital frame
eul_ang0        = [0,0,0];                                                 % [rad]    Euler angles between the orbital 
%                                                                          %          reference frame and the local orbital frame
%                                                                          %          (chaser) -- only if aligned
Pos0            = [x0 y0 z0]';
Vel0            = [x0_dot y0_dot z0_dot]';

deltaVx         = omega_chaser/4*Pos0(3);
deltax          = 3*pi/omega_chaser*deltaVx;
deltax_FD       = 0;

T_1fire         = abs(deltax_FD)/x0_dot;                                   % [s] Istante primo sparo Hohmann
a_Hohmann       = (r_chaser +r_target )/2;
T_Hohmann       = 2*pi*sqrt(a_Hohmann^3/mu);
T_2fire         = T_1fire+ T_Hohmann/2;                                    % [s] Istante secondo sparo Hohmann

% FLAG, Ho aggiunto +10 solo per vedere che succede
T_end_Hohmann = T_2fire +1000 ;    % Usare questo se manovra completa      % [s] Istante di Fine Simulazione Hohmann
% T_end_Hohmann = 2*T_Hohmann; % Usare questo se manovra singola

% Open-Loop Maneuver

deltat_onOP    = m_c0*deltaVx/Total_max_thrust;                            % Tempo di accensione Thruster per la manovra di Hohmann
lambda         = 0.5;                                                      % Per SMC
S              = 0.9;                                                      % Per SMC

tau            = 1.2;
Uon            = 1.25;
Uoff           = 0.55;
deltat_on      = 0.025;
KF             = (Uon-Uoff)/(1-exp(-deltat_on/tau));

%% Hohmann Run
Hohmann_out = sim("Hohmann.slx");
%%

Complete_Maneuvre_time     = Hohmann_out.Positions_IM.time;                % Will be used to store all time instants
Complete_Maneuvre_Position = Hohmann_out.Positions_IM.Data;                % Will be used to store all positions
Complete_Maneuvre_Velocity = Hohmann_out.Velocity_IM.Data;                 % Will be used to store all velocity 

%% Grafici Hohmann
figure
plot(Complete_Maneuvre_Position(:,1), Complete_Maneuvre_Position(:,3) )
set(gca, 'YDir', 'reverse');
set(gca, 'XDir', 'reverse');
xlabel('X LVLH (Vbar)')
ylabel('Z LVLH (Rbar)')
title('Ideal Hohmann Maneuver Positions')
grid on

% Check Tempistiche
figure
plot3(Complete_Maneuvre_Position(:,1), ...
    Complete_Maneuvre_Position(:,3), ...
    Complete_Maneuvre_time )
set(gca, 'YDir', 'reverse');
set(gca, 'XDir', 'reverse');
xlabel('Vbar')
ylabel('Rbar')
zlabel('time')
title('Ideal hohmann Maneuver time evolution')
hold on
plot3(Hohmann_out.Positions_IM.Data(1,1,end), Hohmann_out.Positions_IM.Data(1,3,end), Complete_Maneuvre_time(end) ,'r+')

%% RB

pf_HOH          = Hohmann_out.Positions_IM.Data(end,:,end);                % Posizione termine Hohmann
dpf_HOH         = Hohmann_out.Velocity_IM.Data(end,:,end);                 % Velocità fine Hohmann
%p0_id_RB        = [-3000 0 0];                                            % Posizione teorica inizio manovra
p0_RB           = pf_HOH;                                                  % Posizione effettiva di inizio manovra
deltaz_RB = pf_HOH(3) - p0_RB(3);                                          % [m] Differenza quota RB

%dp0_RB = [3/2*omega_target*deltaz_RB 0 0];                                 % [m/s] Velocità inizion RB 
dp0_RB = dpf_HOH;
pf_id_RB        = [-500 0 0];                                              % Posizione di termine manovra RB

T1_RB           = 10;                                                      % [s] Istante Primo Sparo !Grandezza non nulla solo per evidenziare momento di inizio sparo

T_RB            = (2*pi)/omega_target;                                     % [s] Periodo orbita di trasferimento  
T2_RB           = T1_RB + T_RB/2;                                          % [s] Istante del secondo sparo

delta_x_RB      = pf_id_RB(1) - p0_RB(1);                                  % [m] Variazione di posizione imposta dalla manovra
delta_Vz_RB     = (omega_target/4)*delta_x_RB;                             % [m/s] Variazione istantanea di velocità a seguito dello sparo

T_end_RB = T2_RB + 100; %Usare questo se manovra completa                 % [s] Istante di Fine Simulazione Hohmann
% T_end_RB = 2*T_RB;     % Usare questo se manovra singola
T_end_RB_H = T_end_RB + T_end_Hohmann;

% Definizione Costanti Guida ----------------------------------------------

% SMC
landa_RB        = 1;
S_RB            = 0.5;

%PWPF
Uon             = 1;                                                       % Imposto
Uoff            = 0.7;                                                     % Trovato per tentativi
kf              = 4;                                                       % Valore dettato dalla relazione con K_on e DB
ton             = 0.2;                                                     % [s] Imposto tenendo conto della necessità di incrementarne la durata per ridurre gli spari pur tenendo basso il valore (ipotesi di sparo impulsivo)
tau             = -ton/(log(1-(Uon-Uoff)/(Tmax*kf)));

%% RB Run
Radial_Boost = sim('RB.slx');

%%

% Traiettoria
p_id_RB         = Radial_Boost.p_des_RB;
dp_id_RB        = Radial_Boost.dp_des_RB;
p_RB            = Radial_Boost.p_RB;
dp_RB           = Radial_Boost.dp_RB;

[px_id_RB, py_id_RB, pz_id_RB]      = calcolo_componenti(p_id_RB);         % Posizioni ideali
[dpx_id_RB, dpy_id_RB, dpz_id_RB]   = calcolo_componenti(dp_id_RB);        % Velocità ideali
[px_RB, py_RB, pz_RB]               = calcolo_componenti(p_RB);            % Posizioni effettive
[dpx_RB, dpy_RB, dpz_RB]            = calcolo_componenti(dp_RB);           % Velocità effettive

Actual_RB_Time = Radial_Boost.p_RB.time + T_end_Hohmann;                   % Arithmetic Expansion, runnare con una verisone matlab POST R2016b
Complete_Maneuvre_time     = [Hohmann_out.Positions_IM.time; Actual_RB_Time];    
                                     % Tempo Inizio CA

Complete_Maneuvre_Position = [Hohmann_out.Positions_IM.Data ; [px_RB, py_RB, pz_RB]    ];
Complete_Maneuvre_Velocity = [Hohmann_out.Velocity_IM.Data ; [dpx_RB, dpy_RB, dpz_RB] ];              
%% Grafici Hohmann + RB
figure
plot(Complete_Maneuvre_Position(:,1), Complete_Maneuvre_Position(:,3) )
set(gca, 'YDir', 'reverse');
set(gca, 'XDir', 'reverse');
xlabel('X LVLH (Vbar)')
ylabel('Z LVLH (Rbar)')
title('Ideal Hohmann Maneuver +RB Positions')
grid on

% Check Tempistiche
figure
plot3(Complete_Maneuvre_Position(:,1), ...
    Complete_Maneuvre_Position(:,3), ...
    Complete_Maneuvre_time )
set(gca, 'YDir', 'reverse');
set(gca, 'XDir', 'reverse');
xlabel('Vbar')
ylabel('Rbar')
zlabel('time')
title('Ideal hohmann Maneuver + RB time evolution')

%% CA

pf_RB          = [px_RB(end), py_RB(end), pz_RB(end)];                     % Posizione termine RB
dpf_RB         = [dpx_RB(end), dpy_RB(end), dpz_RB(end)];                  % Velocità termine RB

% Valori iniziali

p0_CA           = pf_RB;                                                   % [m] Posizione
delta_z_CA      = pf_RB(3);
dp0_CA          = dpf_RB;                                                  % [m/s] Velocità

% Valori FInali
pf_id           = [-4 0 0];                                                % [m] Posizione Finale ideale 
% FLAG, a riga 416 la capello mette -2, qui è -4
dpf_id          = [0.05 0 0];                                              % [m/s] Vocistà finale ideale

% spari guida 
deltax_CA       = pf_id(1) - p0_CA(1);
deltaVx_CA      = 0.15;
T1_CA           = 10;
T2_CA           = T1_CA + deltax_CA/deltaVx_CA;

% Valori per la guida

% APF
dx_max          = 0.1;
Ka              = 1;
% SMC (APF)
landa_CA_APF    = 0.01;
S_CA_APF        = 0.01;

% SMC guida
landa_CA        = 1.001;
S_CA            = 0.35;

% PWPF
db              = 0.25;
Uon             = 1;
Uoff            = 0.7;
kf              = 4;
ton             = 0.2;
tau             = -ton/(log(1-(Uon-Uoff)/(Tmax*kf)));

%%
Cone_approach   = sim("Cone_approach.slx");
%%
% traiettorie
p_id_CA         = Cone_approach.p_des_CA;                                  % posizione ideale cone approach
dp_id_CA        = Cone_approach.dp_des_CA;                                 % velocità ideale cone approach
p_CA            = Cone_approach.p_CA;                                      % posizione reale cone approach
dp_CA           = Cone_approach.dp_CA;                                     % velocità reale cone approach

[px_id_CA, py_id_CA, pz_id_CA]      = calcolo_componenti(p_id_CA);
[dpx_id_CA, dpy_id_CA, dpz_id_CA]   = calcolo_componenti(dp_id_CA);
[px_CA, py_CA, pz_CA]               = calcolo_componenti(p_CA);
[dpx_CA, dpy_CA, dpz_CA]            = calcolo_componenti(dp_CA);

Actual_CA_time = Cone_approach.p_CA.time + T_end_RB_H;                     % Arithmetic Expansion, runnare con una verisone matlab POST R2016b

Complete_Maneuvre_time     = [Hohmann_out.Positions_IM.time; Actual_RB_Time; Actual_CA_time];      

Complete_Maneuvre_Position = [Hohmann_out.Positions_IM.Data ; [px_RB, py_RB, pz_RB];   [px_CA, py_CA, pz_CA]  ];
Complete_Maneuvre_Velocity = [Hohmann_out.Velocity_IM.Data ; [dpx_RB, dpy_RB, dpz_RB]; [dpx_CA, dpy_CA, dpz_CA]  ]; 

%% Grafici Hohmann + RB + CA
figure
plot(Complete_Maneuvre_Position(:,1), Complete_Maneuvre_Position(:,3) )
set(gca, 'YDir', 'reverse');
set(gca, 'XDir', 'reverse');
xlabel('X LVLH (Vbar)')
ylabel('Z LVLH (Rbar)')
title('Ideal Hohmann Maneuver +RB + CA Positions')
grid on
hold on
plot(0,0,'r+')
hold off

% Check Tempistiche
figure
plot3(Complete_Maneuvre_Position(:,1), ...
    Complete_Maneuvre_Position(:,3), ...
    Complete_Maneuvre_time )
set(gca, 'YDir', 'reverse');
set(gca, 'XDir', 'reverse');
xlabel('Vbar')
ylabel('Rbar')
zlabel('time')
title('Ideal hohmann Maneuver + RB + CA time evolution')
hold on
plot3(0,0,Complete_Maneuvre_time(end),'r+')
hold off


%% Funzione ausiliaria
function [val_x, val_y, val_z] = calcolo_componenti(timeseries)            % Funzione per estrazione componenti vettori

data            = timeseries.Data;
n               = length(data);
val_x           = data(:,1);
val_y           = data(:,2);
val_z           = data(:,3);

end
