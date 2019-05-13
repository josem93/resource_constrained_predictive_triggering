function [sim,net] = params()
% Function to initialize the simulation, framework, and network parameters.
% 
% Outputs:  sim     Simulation, and framework parameters
%           net     Network parameters    
%
%--------------------------------------------------------------------------

%--------------------------------------------------------------------------
% Simulation Parameters
%--------------------------------------------------------------------------
sim.PROB_TYPE = 'CONTROL';       % problem type {'ESTIMATION','CONTROL'}
sim.TRIG_TYPE = 'ET';            % trigger type {'ET','PT','PPT','ST'}
sim.ET_TYPE = 'ET2';        % ET trigger type {'ET1'=random, 'ET2'=Mamduhi (error-dependent prioritization)}
sim.SIM_TYPE = 'AV';        % simulation type {'STOCHASTIC', 'AV','CP'}

if(strcmp(sim.SIM_TYPE,'CP'))    
    sim.INPUT_NOISE = 0;
    sim.CP_TYPE = 1;        % {0 = Quanser IP02 pend, 1 = new pend}
end

sim.dt = 0.01;             % time step size

% initialize agent models
sim = multi_agent_setup(sim);

%--------------------------------------------------------------------------
% PT Parameters
%--------------------------------------------------------------------------
sim.M = 2;                  %  prediction horizon
sim.FET_TYPE = 'MC';        % probability soln type {'MC','NUM'}
if(strcmp(sim.SIM_TYPE,'STOCHASTIC'))
    sim.delta = 0.02;          % error bound
elseif(strcmp(sim.SIM_TYPE,'AV'))
    sim.delta = 0.01;
elseif(strcmp(sim.SIM_TYPE,'CP'))
    sim.delta = 0.02;
end

if(strcmp(sim.TRIG_TYPE,'ET'))
    sim.P_CONSTRAINT = 0;       
    sim.CAST_FLAG = 0;
elseif(strcmp(sim.TRIG_TYPE,'PT'))    
    sim.P_CONSTRAINT = 1;
    sim.CAST_FLAG = 1;
elseif(strcmp(sim.TRIG_TYPE,'PPT'))
    sim.PT_period = 5;            % compute and send prob at each period
    sim.P_CONSTRAINT = 1;
    sim.CAST_FLAG = 1;
elseif(strcmp(sim.TRIG_TYPE,'ST1'))
    sim.ub = 0.7;                  % ST probability upper bound
    sim.CAST_FLAG = 1;
end

%--------------------------------------------------------------------------
% Network Parameters
%--------------------------------------------------------------------------
net.K = 20;        % # of communication slots

if(strcmp(sim.TRIG_TYPE,'PT') || strcmp(sim.TRIG_TYPE,'PPT') || strcmp(sim.TRIG_TYPE,'ST') || strcmp(sim.TRIG_TYPE,'ST2'))   
    if(strcmp(sim.SIM_TYPE,'STOCHASTIC'))
        net.sched_type = 1;
        net.arbit_type = 2;
        net.p = 0.2;
        net.d = 1;
    elseif(strcmp(sim.SIM_TYPE,'AV'))
        net.sched_type = 0;
        net.arbit_type = 2;
        net.p = 0.2;
        net.d = 1;
    end
end

sim.t = 0:sim.dt:sim.T;     % time vector
