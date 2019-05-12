% Script to run the Multi-agent simulation
%
% sim: structure containing the simulation parameters 
% net: structure containing the network parameters
%
%--------------------------------------------------------------------------

close all;
clear;
clc;

%% Setup multi-agent simulation parameters

sim.N = 5;                 % number of agents
sim.dt = 0.01;              % time step
sim.T = 2;                  % simulation time
sim.t = 0:sim.dt:sim.T;     % time vector

sim.PROB_TYPE = 'CONTROL';       % problem type {'ESTIMATION','CONTROL'}
sim.TRIG_TYPE = 'ET';            % trigger type {'ET','PT','PPT','ST'}
sim.ET_TYPE = 'ET2';
sim.SIM_TYPE = 'AV';     % simulation type {'STOCHASTIC', 'AV','CP'}
sim.INPUT_NOISE = 0;

if(strcmp(sim.SIM_TYPE,'STOCHASTIC'))
    seed = 100;
    rng(seed);
elseif(strcmp(sim.SIM_TYPE,'AV'))
    seed = 100;
    rng(seed);
elseif(strcmp(sim.SIM_TYPE,'CP'))
    seed = 100;
    rng(seed);    
    sim.INPUT_NOISE = 0;
    sim.CP_TYPE = 1;        % {0 = Quanser IP02 pend, 1 = new pend}
end

% initialize agent models
sim = multi_agent_setup(sim);


%% Setup M-step probability parameters
sim.M = 2;                 % number of timesteps to predict ahead
if(strcmp(sim.SIM_TYPE,'STOCHASTIC'))
    sim.FET_TYPE = 'MC';       % probability soln type {'NUM','MC'}
    sim.delta = 0.02;          % error bound
elseif(strcmp(sim.SIM_TYPE,'AV'))
    sim.FET_TYPE = 'MC';
    sim.delta = 0.01;
elseif(strcmp(sim.SIM_TYPE,'CP'))
    sim.FET_TYPE = 'MC';
    if(sim.INPUT_NOISE)
        sim.delta = 0.02;
    else
        sim.delta = 0.02;
    end
end

if(strcmp(sim.TRIG_TYPE,'ET'))
    sim.P_CONSTRAINT = 0;
    sim.CAST_FLAG = 0;
elseif(strcmp(sim.TRIG_TYPE,'PT'))    
    sim.P_CONSTRAINT = 1;
    sim.CAST_FLAG = 1;
elseif(strcmp(sim.TRIG_TYPE,'PPT'))
    sim.PT_period = 5;          % compute and send prob at each period
    sim.P_CONSTRAINT = 1;
    sim.CAST_FLAG = 1;
elseif(strcmp(sim.TRIG_TYPE,'ST1'))
    sim.p = 0.7;                  % ST probability upper bound
    sim.CAST_FLAG = 1;
end

sim.dz0 = floor(200/(2*sim.nx));
sim.Z0_set = -sim.delta:2*sim.delta/(sim.dz0):sim.delta;

% grids for interpolation
sim.Z0_grid = cell(sim.nx,1);
[sim.Z0_grid{:}] = ndgrid(sim.Z0_set);

% Initalize lookup tablesWTB VS The Odd Couple: Movie Trivia Schmoedown
if(strcmp(sim.TRIG_TYPE,'ST1')||strcmp(sim.TRIG_TYPE,'ST2'))
    net.sched_delay = 1;
    sim = Mstep_probability_setup(sim,net.sched_delay);
elseif(sim.M ~= 0)
    sim = Mstep_probability_setup(sim);
end


%% Network config
net.util = zeros(1,length(sim.t));    % network utilization [%]
K = 20;                               % number of set slots we want for simulation

% set network bandwidth [bytes/s for K slots and depending on the scenario]
if(sim.P_CONSTRAINT)
    net.b = (K*4*sim.nx + (4+2)*sim.N)/sim.dt; 
else
    net.b = (K*4*sim.nx + 4*sim.N)/sim.dt;
end

if(strcmp(sim.TRIG_TYPE,'PT') || strcmp(sim.TRIG_TYPE,'PPT') || strcmp(sim.TRIG_TYPE,'ST1') || strcmp(sim.TRIG_TYPE,'ST2'))   
    if(strcmp(sim.SIM_TYPE,'STOCHASTIC'))
        net.sched_type = 1;
        net.arbit_type = 2;
        net.h = 0.2;
        net.d = 1;
    elseif(strcmp(sim.SIM_TYPE,'AV'))
        net.sched_type = 0;
        net.arbit_type = 2;
        net.h = 0.2;%0.2;
        net.d = 1;
    end
else
    net.K = K;
end
net = network_setup(sim,net,sim.N);

%% ET slot allocation method (ET1 in for loop)

% if(strcmp(sim.ET_TYPE,'ET2'))
%     % ET2: random selection of agents (OLD)
%     idx = randperm(sim.N,net.K);
% 
% elseif(strcmp(sim.ET_TYPE,'ET3'))
%     % ET3: leader agents
%     kpl = fix(net.K/sim.lanes);
%     rem = mod(net.K,sim.lanes);
%     extras = zeros(1,sim.lanes);
%     extraIDs = []; ids = [];
%     for n = 1:sim.lanes
%         extras(n) = sim.Nl(n) - kpl;
%         for i = 1:sim.Nl(n)
%             if(i > kpl)
%                 ID = sum(sim.Nl(1:n-1)) + i;
%                 extraIDs = [extraIDs ID];
%             end
%         end
%     end
%     if(rem ~= 0)
%         if(sum(extras) > rem)
%             ids = extraIDs(randperm(numel(extraIDs),rem));
%         end
%     end    
% end


%% Initialize agents
for ID = 1:sim.N
    AGENTS(ID) = Agent(sim,ID);

    % for ET assign slots to first K agents
    if(strcmp(sim.TRIG_TYPE,'ET'))
        if(sim.N > net.K)
            if(strcmp(sim.SIM_TYPE,'STOCHASTIC'))
                if(ID <= net.K)
                    AGENTS(ID).comm_slot = ones(1,length(sim.t));
                end
            elseif(strcmp(sim.SIM_TYPE,'AV'))
%                 if(strcmp(sim.ET_TYPE,'ET2'))
%                     if(sum(idx == ID) == 1)
%                         AGENTS(ID).comm_slot = ones(1,length(sim.t));
%                     end
%                 else
%                     % leader agents assigned slots 
%                     % assumption is that there are always more vehicles in lane
%                     % than slots per lane
%                     if(AGENTS(ID).pLane >= 1 && AGENTS(ID).pLane <= kpl)
%                         AGENTS(ID).comm_slot = ones(1,length(sim.t));
%                     end
%                     if(~isempty(ids))
%                         if(sum(ids == ID))
%                             AGENTS(ID).comm_slot = ones(1,length(sim.t));                    
%                         end
%                     end
%                 
%                 end
                % random selection at each time step inside simulation loop
            end
        else
            AGENTS(ID).comm_slot(:) = 1;
        end
    end

    % compute first ST M value and assign first slot
    if(strcmp(sim.TRIG_TYPE,'ST1') || strcmp(sim.TRIG_TYPE,'ST2'))
        sim.ST_M = find(sim.ST_M_lookup > sim.p,1);
        if(isempty(sim.M))
            sim.ST_M = 30;
        end
        AGENTS(ID).M = sim.ST_M;
        
        if(strcmp(sim.TRIG_TYPE,'ST2'))
            AGENTS(ID).potcomm_slot(sim.ST_M+1) = 1;
        end
    end
end


%% Run simulation for k steps (T runtime)
fprintf('Simulation in progress...');
reverseStr = '';
tSim = tic;
totTies = 0;

if(strcmp(sim.SIM_TYPE,'STOCHASTIC'))
    eps = sqrt(sim.var)*randn(sim.nx,sim.N,length(sim.t));
elseif(strcmp(sim.SIM_TYPE,'AV'))
    eps(1,:,:) = sqrt(sim.var2)*randn(1,sim.N,length(sim.t));
    eps(2,:,:) = sqrt(sim.var2)*randn(1,sim.N,length(sim.t));
    eps(3,:,:) = sqrt(sim.var2)*randn(1,sim.N,length(sim.t));
    eps(4,:,:) = sqrt(sim.var2)*randn(1,sim.N,length(sim.t));
end

for k = 2:length(sim.t)   
    
    sim.eps = eps(:,:,k);
    
    % randomly assign slots for ET
    if(strcmp(sim.TRIG_TYPE,'ET1'))
        if(sim.N > net.K)
            idx = randperm(sim.N,net.K);    
            for ID = 1:sim.N
                if(sum(idx == ID) == 1)
                    AGENTS(ID).comm_slot(k) = 1;
                end           
            end
        end
    end
    
    % run process for all agents
    arrayfun( @(obj) obj.process(sim,net,k), AGENTS);    
      
    if(strcmp(sim.SIM_TYPE,'AV'))
        psent = [AGENTS.psent];
    
        for n = 1:sim.lanes
            for i = 1:sim.Nl(n)
                ID = sum(sim.Nl(1:n-1)) + i;
                if(mod(ID,sim.Nl(n)) ~= 1)
                    AGENTS(ID).ppreced = psent(ID-1);
                end
            end
        end
    end
    
    % reset agent predictions given communications
    commDecisions = [AGENTS.comm_trig_curr];
    for ID = 1:sim.N
        AGENTS(ID).comm_agents = commDecisions;
    end
    
    % create array for states of communicating agents;
    sentStates = [AGENTS.sent_state];
    for ID = 1:sim.N
        AGENTS(ID).all_sent_states = sentStates;
    end
    
    % run predictor blocks for all agents
    arrayfun( @(obj) obj.predictors(sim,k), AGENTS);
    
    % run error for all agents
    arrayfun( @(obj) obj.error(k), AGENTS);

    if(strcmp(sim.PROB_TYPE,'CONTROL'))
        % run controller for all agents
        arrayfun( @(obj) obj.controller(sim,k), AGENTS);
    end
    
    
    if(strcmp(sim.TRIG_TYPE,'ET'))
        if(strcmp(sim.ET_TYPE,'ET1'))
            Np = 0;
            % Communicate
            arrayfun( @(obj) obj.ET_comm(sim,k), AGENTS);
        elseif(strcmp(sim.ET_TYPE,'ET2'))
            Np = sim.N;
            % all agent errors
            errors = [AGENTS.z_curr];

            % priority measure
            znorms = sqrt(sum(errors.*errors));
            
            % send to Network manager
            slots = NetworkManager_ET(sim,net,znorms);
            args = num2cell(slots);
            [AGENTS.comm_slot_mstep] = deal(args{:});
            
            % Schedule slots
            arrayfun( @(obj) obj.schedule_slot(sim,k), AGENTS);
        end
        
    elseif(strcmp(sim.TRIG_TYPE,'PT'))

        % all agent errors
        errors = [AGENTS.z_curr];

        % get m-step probabilities for all agents
        H = Mstep_probability(sim,sim.TRIG_TYPE,sim.M,errors);    
        for ID = 1:sim.N
            AGENTS(ID).H = H(:,ID);
        end

        % compute the communication probability at time k+M
        if(sim.M == 0)
            mstep_probs = H;
        else
            arrayfun( @(obj) obj.pred_comm_prob(sim,k), AGENTS);
            mstep_probs = [AGENTS.comm_prob_mstep];
        end
            
        % send to Network Manager and assign slots
        if(sim.P_CONSTRAINT)
            mstep_send = []; ids = [];
            for ID = 1:sim.N
                if(mstep_probs(ID) > net.h/net.d)
                    mstep_send = [mstep_send, mstep_probs(ID)];
                    ids = [ids ID];
                end
            end
            Np = length(mstep_send);
            if(sim.CAST_FLAG)
                mstep_send = uint8(100*mstep_send);
            end
            if(Np == sim.N)
                [slots,ties] = NetworkManager(sim,net,mstep_send);              
            else
                [slots,ties] = NetworkManager(sim,net,mstep_send,ids);   
            end

        else
            Np = sim.N;
            if(sim.CAST_FLAG)
                mstep_send = uint8(100*mstep_probs);
            else
                mstep_send = mstep_probs;
            end
            [slots,ties] = NetworkManager(sim,net,mstep_send);
        end
        
        args = num2cell(slots);
        [AGENTS.comm_slot_mstep] = deal(args{:});

        % Schedule slots
        arrayfun( @(obj) obj.schedule_slot(sim,k), AGENTS);
     
        
    elseif(strcmp(sim.TRIG_TYPE,'PPT'))
        if(mod(k,sim.PT_period) == 0)
            
            % all agent errors
            errors = [AGENTS.z_curr];
            if(strcmp(sim.SIM_TYPE,'AV'))
                errors = errors(1,:);
            end
            
            % get m-step probabilities for all agents
            H = Mstep_probability(sim,sim.TRIG_TYPE,sim.M,errors);    
            for ID = 1:sim.N
                AGENTS(ID).H = H(:,ID);
            end

            % compute the communication probability at time k+M
            arrayfun( @(obj) obj.pred_comm_prob(sim,k), AGENTS);
            mstep_probs = [AGENTS.comm_prob_mstep];
            
            % send to Network Manager and assign slots
            if(sim.P_CONSTRAINT)
                mstep_send = []; ids = [];
                for ID = 1:sim.N
                    if(mstep_probs(ID) > net.h/net.d)
                        mstep_send = [mstep_send, mstep_probs(ID)];
                        ids = [ids ID];
                    end
                end
                Np = length(mstep_send);
                if(sim.CAST_FLAG)
                    mstep_send = uint8(100*mstep_send);
                end
                if(Np == sim.N)
                    [slots,ties] = NetworkManager(sim,net,mstep_send);              
                else
                    [slots,ties] = NetworkManager(sim,net,mstep_send,ids);   
                end

            else
                Np = sim.N;
                if(sim.CAST_FLAG)
                    mstep_send = uint8(100*mstep_probs);
                else
                    mstep_send = mstep_probs;
                end
                [slots,ties] = NetworkManager(sim,net,mstep_send);
            end

            args = num2cell(slots);
            [AGENTS.comm_slot_mstep] = deal(args{:});

            % Schedule slots
            arrayfun( @(obj) obj.schedule_slot(sim,k), AGENTS);

        else
            Np = 0;
        end
        
    elseif(strcmp(sim.TRIG_TYPE,'ST1'))

        send = []; ids = [];
        for ID = 1:sim.N
            if(k <= length(sim.t)-AGENTS(ID).M)
                if(strcmp(AGENTS(ID).trig_type,'PT'))
                    error = AGENTS(ID).z_curr;
                    prob = Mstep_probability(sim,AGENTS(ID).trig_type,AGENTS(ID).M,error);
                    send = [send, prob(end)];
                    ids = [ids, ID];
                    %ADD prediction horizon
                
                elseif(strcmp(AGENTS(ID).trig_type,'ST1'))
                    if(k == 2)
                        send = [send AGENTS(ID).M];
                        ids = 1:sim.N;
                    else
                        if(AGENTS(ID).comm_trig(k))
                            send = [send AGENTS(ID).M];
                            ids = [ids ID];
                        end
                    end
                end
            end
        end
        
        % send to Network Manager and assign slots
        if(~isempty(send))
            Np = length(send);
            [slots,~] = NetworkManager(sim,net,send,ids);

            for m = 1:size(slots,2)
                for ID = 1:sim.N
                    % if agent switches to PT
                    if(strcmp(AGENTS(ID).trig_type,'PT'))
                        if(m == sim.M)
                            if(k + AGENTS(ID).M <= length(sim.t))
                                AGENTS(ID).comm_slot(k+AGENTS(ID).M) = slots(ID,m);
                            end
                        end
                    elseif(strcmp(AGENTS(ID).trig_type,'ST1'))
                        % if it hasnt already been given a slot
                        if(~AGENTS(ID).comm_slot(k+m))
                            AGENTS(ID).comm_slot(k+m) = slots(ID,m);
                        end 
                    end
                end
            end
        else
            Np = 0;
        end
        % Communicate
        arrayfun( @(obj) obj.ST1_comm(sim,sim.ST_M,k), AGENTS);

        
    elseif(strcmp(sim.TRIG_TYPE,'ST2'))
        
        % get agent error that may communicate at next time step
        mstep_probs = [];
        ids = [];
        for ID = 1:sim.N
            if(k <= length(sim.t)-net.sched_delay)
                if(strcmp(AGENTS(ID).trig_type,'PT'))
                    error = AGENTS(ID).z_curr;
                    mstep_probs = [mstep_probs, Mstep_probability(sim,AGENTS(ID).trig_type,AGENTS(ID).M,error)];
                    ids = [ids, ID];
                elseif(strcmp(AGENTS(ID).trig_type,'ST2'))
                    if(AGENTS(ID).pot_comm_slot(k+net.sched_delay))
                        error = AGENTS(ID).z_curr;
                        mstep_probs = [mstep_probs, Mstep_probability(sim,AGENTS(ID).trig_type,AGENTS(ID).M,error)];
                        ids = [ids, ID];
                    end
                end
            end
        end        

        if(~isempty(mstep_probs))
            % check if # of time steps need to be assigned is > than M
            if(ceil(sim.N/net.K) > sim.ST_M)
                i = 1;
                nA_sched = 0;
                NAS = [];
                while(true)
                    nA_sched = 0;
                    for ID = 1:sim.N
                        if(AGENTS(ID).comm_slot(k+i))
                            nA_sched = nA_sched + 1;
                        end
                    end
                    NAS = [NAS nA_sched];
                    if(nA_sched < net.K)
                        break;
                    end
                    i = i+1;
                end
            else
                NAS = 0;
            end
            % send to Network Manager and assign slots
            Np = length(mstep_probs);
            [slots,~] = NetworkManager(sim,net,mstep_probs,ids,NAS(end));
            if(k + size(slots,2) <= length(sim.t))
                for m = 1:size(slots,2)
                    for ID = 1:sim.N
                        % if agent switches to PT
                        if(slots(ID,1) == -1)
                            AGENTS(ID).trig_type = 'PT';
                            AGENTS(ID).M = 2;
                            if(k + AGENTS(ID).M <= length(sim.t))
                                AGENTS(ID).comm_slot(k+AGENTS(ID).M) = 0;
                            end
                        else
                            if(strcmp(AGENTS(ID).trig_type,'PT'))
                                if(m == 1)
                                    if(k + AGENTS(ID).M <= length(sim.t))
                                        AGENTS(ID).comm_slot(k+AGENTS(ID).M) = slots(ID,1);
                                    end
                                end
                            elseif(strcmp(AGENTS(ID).trig_type,'ST2'))
                                % if it hasnt already been given a slot
                                if(~AGENTS(ID).comm_slot(k+net.sched_delay-1+length(NAS)-1+m))
                                    AGENTS(ID).comm_slot(k+net.sched_delay-1+length(NAS)-1+m) = slots(ID,m);
                                end 
                            end
                        end
                    end
                end
            end
        else
            Np = 0;
        end
             
        % Communicate
        arrayfun( @(obj) obj.ST_comm(sim.t,sim.ST_M,k), AGENTS);
        
    end

    % network utilization
    Nc = sum(commDecisions);
    if(sim.P_CONSTRAINT||strcmp(sim.TRIG_TYPE,'ST1')||strcmp(sim.TRIG_TYPE,'ST2'))
        if(sim.CAST_FLAG)
            net.util(k) = 100*((1+2)*Np + 4*sim.nx*Nc)/net.capacity;                        
        else
            net.util(k) = 100*((4+2)*Np + 4*sim.nx*Nc)/net.capacity;                
        end
    else
        if(sim.CAST_FLAG)
            net.util(k) = 100*(Np + 4*sim.nx*Nc)/net.capacity;        
        else
            net.util(k) = 100*(4*Np + 4*sim.nx*Nc)/net.capacity;                
        end
    end
    
    % total number of ties
    if(strcmp(sim.TRIG_TYPE,'PT'))
        totTies = totTies +ties;    
    end
    
    % display percentage completed
    percentDone = floor(100*k/length(sim.t));
    msg = sprintf('%i', percentDone);
    fprintf([reverseStr, msg]);
    fprintf('%%');
    reverseStr = repmat(sprintf('\b'), 1, length(msg)+1);
end
sim.runtime = toc(tSim);
fprintf([reverseStr, 'Done\n\n']);
fprintf(['\t Simulation completed in ',num2str(sim.runtime),' seconds\n\n']);


%% POSTPROCESSING

disp(['Number of time steps w/ critical ties = ',  num2str(totTies)]);
disp(['Percentage = ',  num2str(totTies/length(sim.t))]);

% Check to see if any agent errors are above delta
numAbove = 0;
for i = 1:sim.N
    a(i) = AGENTS(i).check_error(sim);
    numAbove = numAbove + a(i);
end
if(numAbove == 0)
    disp('All agent estimation errors below threshold');
else
    disp('An agent estimation errors exceeds threshold');
end

% Plot
% pos = [759 302 813 645];
% plot_settings(18,1.5);

if(strcmp(sim.SIM_TYPE,'STOCHASTIC'))
    if(strcmp(sim.PROB_TYPE,'ESTIMATION'))
        for ID = 1:10
        %     AGENTS(i).plotStates(sim.nx,sim.t,sim.delta);
            AGENTS(ID).plotEstError(sim.nx,sim.t,sim.delta);
        end
 
    elseif(strcmp(sim.PROB_TYPE,'CONTROL'))
        for ID = 1:10
            AGENTS(i).plotStates(sim.nx,sim.t,sim.delta);
%             AGENTS(ID).plotCtrlError(sim);
        end
    end
    figure;
    stairs(sim.t,net.util);
    xlabel('Time [s]');
    ylabel('Network Utilization [%]');
    
elseif(strcmp(sim.SIM_TYPE,'AV'))
    
    % compute inter-vehicle distance (d)
    for n = 1:sim.lanes
        for i = 1:sim.Nl(n)   
            if(i == 1)
                for k = 2:length(sim.t)
                    % distance to reference vehicle
                    AGENTS(i).d(k) = sim.pref(k) - AGENTS(i).px(k) - sim.L(i);
                end
            else
                ID = sum(sim.Nl(1:n-1)) + i;
                for k = 2:length(sim.t)
                    AGENTS(ID).d(k) = AGENTS(ID-1).px(k) - AGENTS(ID).px(k) - sim.L(ID);
                    AGENTS(ID).dr(k) = sim.r(ID) + sim.h*AGENTS(ID).v(k);
%                     AGENTS(ID).d(k) = AGENTS(ID).d(k-1) + sim.dt*(AGENTS(ID-1).v(k-1)-AGENTS(ID).v(k-1));
                end
            end
        end
    end
    
    % Check for crash
    crash = 0;
    pos_all = [AGENTS.px];
    pos_all = reshape(pos_all,length(sim.t),sim.N);
    pos_all = pos_all';
    dp = abs(diff(pos_all));
    for i = 1:sim.N
        if(i ~= 1)
            for j = 1:size(dp,2)
                if(dp(i-1,j) <= sim.L(i-1))
                    crash = 1;
                    disp([num2str(i-1),',',num2str(j)]);
                    disp('CRASH!!!');
                    break;
                end
            end
        end
        if(crash)
            break;
        end
    end
    
    % compute mean control error norm
    for ID = 1:sim.N
        ebar(ID) = AGENTS(ID).mean_ctrlError_norm(sim);
    end
    enorm = mean(ebar)

    % vehicle velocities
    for n = 1:sim.lanes
        figure;
%         figure('Position',pos);
        hold on;grid on;box on;
        title(['Lane ',num2str(n), ' - ',sim.TRIG_TYPE,' K = ',num2str(net.K),' Norm = ',num2str(enorm)]);
        for i = 1:sim.Nl(n)+1
            if(i == 1)
                plot(sim.t,sim.vref(n,:));
                leg{i} = ['vref'];
            else
                ID = sum(sim.Nl(1:n-1)) + (i-1);            
                plot(sim.t,AGENTS(ID).v);
%                 plot(sim.t,AGENTS(ID).x(1,:));
%                 plot(sim.t,AGENTS(ID).xhat(1,:,2));
                leg{i} = ['v',num2str(i-1)];
            end
        end
        xlabel('Time [s]');
        ylabel('Velocity [m/s]');
        l = legend(leg);
        l.Location = 'best';
        clear leg;
    end
    
    % vehicle position
%     for n = 1:sim.lanes
%         figure;
%         hold on;grid on;box on;
%         title(['Lane',num2str(n)]);
%         for i = 1:sim.Nl(n)+1
%             if(i == 1)
%                 plot(sim.t,sim.pref(n,:));
%             else
%                 ID = sum(sim.Nl(1:n-1)) + (i-1);
%                 plot(sim.t,AGENTS(ID).px);
%             end
%         end
%     end

    % inter-vehice distance
    for n = 1:sim.lanes
        figure;
%         figure('Position',pos);
        hold on;grid on;box on;
        title(['Lane ',num2str(n), ' - ',sim.TRIG_TYPE,' K = ',num2str(net.K),' Norm = ',num2str(enorm)]);
        plot(sim.t,AGENTS(2).dr,'g--');
        leg{1} = ['desired'];
        for i = 1:sim.Nl(n)
            if(i ~= 1)
                ID = sum(sim.Nl(1:n-1)) + i;             
                plot(sim.t,AGENTS(ID).d);
                leg{i} = ['v',num2str(i-1),'v',num2str(i)];
            end
        end
        xlabel('Time [s]');
        ylabel('Inter-vehicle Distance [m]');
        l = legend(leg);
        l.Location = 'best';
        clear leg;
    end
    
%     for ID = 1:sim.N
%     %     AGENTS(i).plotStates(sim.nx,sim.t,sim.delta);
%         AGENTS(ID).plotEstError(sim.nx,sim.t,sim.delta);
%     end
    
%     figure;
%     stairs(sim.t,net.util);
%     xlabel('Time [s]');
%     ylabel('Network Utilization [%]');

end

