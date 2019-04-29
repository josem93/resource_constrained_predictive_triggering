%%
%
%--------------------------------------------------------------------------

fprintf('Initializing vehicle parameters...');


%--------------------------------------------------------------------------
% Mass & Length parameters
%--------------------------------------------------------------------------

m  = 1300;          % vehicle mass [kg]
l  = 4;             % vehicle length [m]
g  = 9.81;          % gravity acceleration [m/s^2]
lf = 1.1;           % Distance of front wheel and CG [m]
lr = 1.6;           % Distance of front wheel and CG [m]
h  = 0.5;           % height of vehicle CG [m]
Jz = 2000;          % Inertia around cg [kgm^2]

%--------------------------------------------------------------------------
% Aerodynamic parameters
%--------------------------------------------------------------------------
Cd  = 0.3;                          % drag coefficient []
rho = 1.225;                        % air density [kg/m^3]
A   = 1.6 + 0.00056*(m - 765);      % frontal area [m^2]
Vw  = 0;                            % headwind speed [m/s^2]

%--------------------------------------------------------------------------
% Engine parameters
%--------------------------------------------------------------------------
% tau_thr_max = [1,2,3,4,5,6];        % for each gear
tau_thr_max = 260;                  % [Nm]

%--------------------------------------------------------------------------
% Braking parameters
%--------------------------------------------------------------------------
Cbrk = .1;            % braking constant?

%--------------------------------------------------------------------------
% Transmission parameters
%--------------------------------------------------------------------------
xg = [3.643 2.080 1.361 1.024 0.830 0.686];     % gear ratio
xd = 4.105;                                     % differential ratio
nt = 0.7;                                       % transmission efficiency

%--------------------------------------------------------------------------
% Tire parameters
%--------------------------------------------------------------------------
Rw = 0.4;           % wheel radius [m]
mw = 9.5;           % wheel mass [kg]
Jw = mw*Rw^2;       % 2-wheel inertia [kgm^2]

tau_bs = 0.2;       % brake pressure lag [s]
Kb = 10;            % pressure/torque conversion [Nm/bar]
Kc = 1;             % pressure gain

B = 10;             %  Pacejka model coefficients
C = 1.9;
D = 1;
E = 0.97;

% Cs_f = ;                        % tire slip coefficient [N/deg]
Ca_f = 100000*(pi/180);         % front tire sideslip coefficient [N/deg]
Ca_r = 120000*(pi/180);         % rear tire sideslip coefficient [N/deg]
Cr   = 0.015;                   % rolling resistance coefficient


%--------------------------------------------------------------------------
% Road parameters
%--------------------------------------------------------------------------
theta = 0;                  % road inclination
lanes  = 3;                 % number of lanes
lwidth = 3.7;               % lane width [m]

ylc = zeros(1,lanes);       % lane center positions [m]
for i = 1:lanes
    ylc(i) = lwidth/2 + (i-1)*lwidth;
end


fprintf('done\n');