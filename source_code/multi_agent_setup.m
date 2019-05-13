function sim = multi_agent_setup(varargin)
% Setup function for multi-agent system simulations
% Initializes the model matrices and input matrices for either the 
%   stochastic processes, autonomous vehicles or cart-pole experiments

%--------------------------------------------------------------------------
sim = varargin{1};
if(nargin >= 2)
    sim.N = max(varargin{2});
    
    if(nargin > 2 && strcmp(sim.SIM_TYPE,'AV'))
        sim.lanes = max(varargin{3});
        sim.Nl = varargin{4}(:,end)';
    end
end


%% Stochastic Process Simulation

if(strcmp(sim.SIM_TYPE,'STOCHASTIC'))
addpath(genpath(strcat(pwd,'/Stochastic Processes')));

sim.N = 5;          % number of agents
sim.T = 5;          % simulation time

sim.nx = 1;         % number of states
sim.nu = 1;         % number of inputs
sim.ny = 1;         % number of outputs

fprintf('Initalizing stochastic process models...');

% Homogenous system
sim.A = zeros(sim.nx,sim.nx);
sim.x0 = zeros(sim.nx,1,sim.N);
sim.Ahat = zeros(sim.nx,sim.nx,sim.N);
sim.eps = zeros(sim.nx,1);

% initialize model and noise covariance matrix
% stable case
rss = drss(sim.nx);
sim.A = rss.A;
sim.var = 8.1e-5;

% unstable case (random)
% sim.A(1:1+size(sim.A,1):end) = 1;
% unstable case
% d = 1.1*ones(sim.nx,1);
% sim.A = diag(d);
% sim.var = 2.5e-5;

sim.Q = diag(sim.var*ones(sim.nx,1));          % diagonal covariance matrix
sim.eps = sqrt(sim.Q)*randn(sim.nx,1);         % N(0,2.5e-5) distribution

for agent = 1:sim.N
    % initialize prediction Ahat matrix
    sim.Ahat(:,:,agent) = sim.A;
       
    % initialize agent initial state
    sim.x0(:,:,agent) = randn(sim.nx,1);          % N(1,1) distribution
end
clear agent;

if(strcmp(sim.PROB_TYPE,'CONTROL'))  
    sim.B = 0.01*ones(sim.nx,sim.nu);
%     for i = 1:sim.nu
%         sim.B(i,i) = 0.01;
%     end
    q = 1; r = 0.1;
    sim.Q = diag(q*ones(sim.nx,1));
    sim.R = diag(r*ones(sim.nu,1));
end

% adjacency matrix for communication
sim.Adj = ones(sim.N,sim.N);            % all agents comm. to each other

%% Heterogenous system
%
% for agent = 1:sim.N
%     % initialize process A matrix
%     d = sqrt(6.25e-4)*randn(sim.nx,1) + 0.9;       % N(0.9, 6.25e-4) distribution for stable A
%     sim.A(:,:,agent) = diag(d);
%     % initialize prediction Ahat matrix
%     r = 0.1*rand(sim.nx,1);
%     sim.Ahat = sim.A - diag(r);
%     
%     % initialize process noise covariance matrix
%     sim.eps(:,:,agent) = sqrt(2.5e-5)*randn(sim.nx,1);      % N(0,2.5e-5) distribution
%     
%     % initialize agent initial state
%     sim.x0(:,:,agent) = randn(sim.nx,1)+1;          % N(1,1) distribution
% end

fprintf('done\n\n');
end



%% Autonomous Vehicle Simulation

if(strcmp(sim.SIM_TYPE, 'AV'))
addpath(genpath(strcat(pwd,'/Autonomous Vehicles')));

fprintf('Initalizing vehicle model...');
% init_av_params;

sim.T = 120;               % simulation time [s]
% sim.T = 450;               % simulation time [s]
sim.t = 0:sim.dt:sim.T;    % time vector

if(nargin < 2)
%     sim.lanes = 1;             % # lanes
    sim.lanes = 5;             % # lanes
    sim.Nl = 8*ones(1,sim.lanes);   % # vehicles per lane
    sim.N = sum(sim.Nl);     % number of agents
end

sim.lwidth = 3.7;          % lane width [m]
ylc = zeros(1,sim.lanes);      % lane center positions [m]
for i = 1:sim.lanes
    ylc(i) = sim.lwidth/2 + (i-1)*sim.lwidth;
end

sim.h = 0.7;               % time gap [s]
sim.tau = 0.1;            % engine driveline time constant [s]
sim.r = 2.5*ones(sim.N,1);  % standstill distance [m] (set to 0 if highway simulation)
sim.L = 4*ones(sim.N,1);    % vehicle lengths

% controller gains
kp = 0.2;
kd = 0.7;
kdd = 0;
if((1+kdd)*kd <= kp*sim.tau)
    error('Unstable controller');
end
sim.K = [kp kd kdd];

% Assign memory for variables
sim.posx = zeros(sim.N,length(sim.t));      % x-pos matrix (all agents, all times)
sim.posy = zeros(sim.N,length(sim.t));      % y-pos matrix (all agents, all times)
sim.v = zeros(sim.N,length(sim.t));         % velocity matrix (all agents, all times)
sim.a = zeros(sim.N,length(sim.t));         % accel matrix (all agents, all times)
sim.d = zeros(sim.N,length(sim.t));         % distance matrix (all agents, all times)
sim.dr = zeros(sim.N,length(sim.t));        % desired distance (all agents, times)
sim.e = zeros(sim.N,length(sim.t));         % error matrix (all agents, all times)
sim.u = zeros(sim.N,length(sim.t));         % desired acceleration (all agents,times)

for n = 1:sim.lanes
    for i = 1:sim.Nl(n)
        ID = sum(sim.Nl(1:n-1)) + i;
        sim.posy(ID,:) = ylc(n)*ones(1,length(sim.t));      % y-pos matrix (all agents, all times)   
    end
end

sim.pref = zeros(sim.lanes,length(sim.t));
sim.vref = zeros(sim.lanes,length(sim.t));
sim.aref = zeros(sim.lanes,length(sim.t));
sim.adotref = zeros(sim.lanes,length(sim.t));
sim.uref = zeros(sim.lanes,length(sim.t));

sim.var1 = 2.5e-5;%6.4e-5;
sim.var2 = 9e-6;%1e-6;
sim.var3 = eps;

% adjacency matrix for communication
% sim.Adj = zeros(sim.N);

% all vehicles in lane communicate to each other
% str = '';
% for n = 1:sim.lanes
%     a{n} = ones(sim.Nl(n));
%     str = [str,'a{',num2str(n),'}'];
%     if(n ~= sim.lanes)
%         str = [str,','];
%     end
% end
% sim.Adj = eval(['blkdiag(',str,')']);
% sim.deg = sum(sim.Adj,2);

% distributed gains
% sim.F = 18*ones(1,3);%sim.K;
% sim.G = 0.01;

% if all gents communicate to each other
% sim.sumAF1 = (1/sim.deg(1))*sim.N*sim.F(1);
% sim.sumAF2 = (1/sim.deg(1))*sim.N*sim.F(2);
% sim.sumAF3 = (1/sim.deg(1))*sim.N*sim.F(3);
% sim.sumAG = (1/sim.deg(1))*sim.N*sim.G;

% Method with modified PT framework (predict u) - Distributed CACC
% sim.A = [   0           1           0;
%             0           0           1;
%             -(sim.sumAF1/sim.tau)   -(sim.sumAF2/sim.tau)   -((1+sim.sumAF3)/sim.tau)];
% 
% sim.Au = 1 - (sim.dt/sim.h)*(1-sim.sumAG);
% sim.nx = size(sim.A,2);             % system dimension
% sim.nu = 1;
% sim.A = eye(sim.nx) + sim.dt*sim.A;
% 
% sim.Q = diag([sim.var1;sim.var3;sim.var2;sim.F(1)*sim.var1+sim.F(2)*sim.var2]);


% Method with origianl PT framework (predict full state) - Regular CACC
sim.A = [   0   1   0   0;
            0   0   1   0;
            0   0   -1/sim.tau   0;
            0   0   0   -1/sim.h];
sim.B = [0; 0; -1/sim.tau; 1/sim.h];
sim.Bff = [0; 0; 1/sim.tau; 0];
sim.nx = size(sim.A,2);             % system dimension
sim.nu = size(sim.B,2);
% discrete
sim.A = eye(sim.nx) + sim.dt*sim.A;
sim.B = sim.dt*sim.B;
sim.Bff = sim.dt*sim.Bff;

% diagonal covariance matrix
sim.Q = diag([sim.var2;sim.var2;sim.var2;kp*sim.var2+kd*sim.var2]);


for agent = 1:sim.N
    % initialize prediction Ahat matrix
    sim.Ahat(:,:,agent) = sim.A;
end
clear agent;

% Initialize position and velocity of vehicles
v_init = zeros(sim.N,1);            % initial vehicle velocities [m/s]
d_init = 5*ones(sim.lanes,1);       % initial inter-vehicle distance [m]

sim.x0 = zeros(sim.nx,1,sim.N);
sim.x = zeros(sim.nx,length(sim.t),sim.N);
sim.xref = zeros(sim.nx,length(sim.t),sim.lanes);

for n = 1:sim.lanes
    for i = 1:sim.Nl(n)+1
        if(i == 1)
            % initial position of reference vehicle
            sim.pref(n,1) = sim.Nl(n)*(d_init(n)+sim.h*sim.v(1,1)+sim.L(1));

            % set velocity profile for virtual reference vehicle
            sim.vref(n,:) = setLeadVelocity(sim.t,sim.dt);

            sim.pref(n,:) = cumtrapz(sim.t,sim.vref(n,:)) + sim.pref(n,1);
            sim.aref(n,:) = diff([eps sim.vref(n,:)])/sim.dt;
            sim.adotref(n,:) = diff([eps sim.aref(n,:)])/sim.dt;
            sim.uref(n,:) = sim.tau*sim.adotref(n,:) + sim.aref(n,:);
            sim.xref(4,:,n) = sim.uref(n,:);

        else
            ID = sum(sim.Nl(1:n-1)) + (i-1);
            sim.v(ID,1) = v_init(ID);
            if(sim.v(ID,1) == 0)
                sim.dr(ID,1) = sim.r(ID);
            else
                sim.dr(ID,1) = sim.h*sim.v(ID,1);
            end
            if(mod(ID,sim.Nl(n)) == 1)
                sim.posx(ID,1) = sim.pref(n,1) - d_init(n) - sim.L(ID);
                sim.d(ID,1) = sim.pref(n,1) - sim.posx(ID,1) - sim.L(ID);           
            else
                sim.posx(ID,1) = sim.posx(ID-1,1) - d_init(n) - sim.L(ID);
                sim.d(ID,1) = sim.posx(ID-1,1) - sim.posx(ID,1) - sim.L(ID); 
            end
            sim.e(ID,1) = sim.d(ID,1) - sim.dr(ID,1);
            sim.x0(1,1,ID) = sim.e(ID,1);
            sim.x(:,1,ID) = sim.x0(:,1,ID);
        end
    end
end

fprintf('Done\n\n');
end



%% Cart-Pole Simulation
if(strcmp(sim.SIM_TYPE, 'CP'))
addpath(genpath(strcat(pwd,'/Cart-Pole')));

sim.N = 10;                 % number of agents
sim.T = 60;                 % experiment time [s]

% simulated agents
if(sim.CP_TYPE)
    sim.mass = 89;             % mass on pole

    for i = 1:10
        file1 = ['A',num2str(i),'_',num2str(sim.mass),'g.mat'];
        file2 = ['B',num2str(i),'_',num2str(sim.mass),'g.mat'];

        f = load(file1);
        A(:,:,i) = f.A_new;
        f = load(file2);
        B(:,i) = f.B_new;
    end
    fclose('all');

    Ad_sysID = mean(A,3);
    Bd_sysID = mean(B,2);
    sim.A = Ad_sysID;
    sim.B = Bd_sysID;
    sim.var = 2.5e-5;
else
    load('CP_model.mat');
    sim.A = Ad_model;
    sim.B = Bd_model;
    sim.var = 2.5e-5; %6.4e-5; %1e-4;
end
sim.nx = size(sim.A,1);     % number of states
sim.nu = size(sim.B,2);     % number of inpnuts
sim.ny = sim.nx;            % number of outputs

Cd = eye(sim.nx);
Dd = zeros(sim.nx,sim.nu);
sysd = ss(sim.A,sim.B,Cd,Dd,0.001);
sysd_10 = d2d(sysd,0.01);
sim.A = sysd_10.A;
sim.B = sysd_10.B;

sim.Q = diag(sim.var*ones(sim.nx,1));          % diagonal covariance matrix

end