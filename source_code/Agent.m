% Agent class containing the fields and methods needed to run the ET, PT,ST
% blocks

classdef Agent < handle
   
properties
    ID              % agent identification number
    trig_type       % triggering type
    A               % process model of current agent  [nx x nx]
    Ahat            % prediction models of all agents [nx x nx x N]
    B               % input matrix                    [nx x nu]
    Bff             % feedforward matrix (for AV)     [nx x 1]
    x0              % initial state of current agent  [nx x 1]
    x               % current state of current agent  [nx x T]
    xhat            % predicted states of all agents  [nx x T x N]
    z               % estimation error of agent       [nx x T]
    z_curr          % current error of agent          [nx x 1]
    u               % control input for process       [nu x T]
    e               % control error of agent          [nx x T]
    lane            % lane of vehicle                 [1 x 1]
    pLane           % vehicle position in lane        [1 x 1]
    a               % vehicle acceleration            [1 x T]
    v               % vehicle velocity                [1 x T]
    px              % vehicle x-position              [1 x T]
    py              % vehicle y-position              [1 x T]
    d               % distance to preceding           [1 x T]
    dr              % desired distance to preceding   [1 x T]
    se              % (AV) spacing error              [1 x T]
    ppreced         % preceding vehicles position     [1 x T]
    psent           % position sent at time k         [1 x 1]
    ad              % desired acceleration            [1 x T]
    chi             % control input for AV            [nu x T]
    adtilde         % desired acc. of i-1 seen by i   [1 x T]
    F               % controller gain                 [nu x nx]
    H               % 1-M step exit probability from Z(k) [M x 1]
    M               % prediction horizon (variable for ST)
    comm_prob       % comm prob in prediction horizon [1 x T]
    comm_prob_mstep % M-step communication probability
    comm_slot       % assigned communication slot     [T x {0,1}]
    comm_slot_mstep % M-step slot assignment          [{0,1}]
    pot_comm_slot   % potential comm slot for ST      [T x {0,1}]
    comm_trig       % communication trigger           [T x {0,1}]
    comm_trig_curr  % current triggering decision     [{0,1}]
    comm_agents     % comm decision of all agents at time k  [{0,1} x N]
    sent_state      % state sent by current agent comm  [nx x 1]
    all_sent_states % states sent by all agents at time k [nx x N]
end

methods
    % Constructor
    function obj = Agent(sim,ID)
        if(nargin > 0)
            if(isnumeric(sim.A) && isnumeric(sim.Ahat) && isnumeric(sim.x0))
                if(size(sim.x0,1) == length(sim.A))
                    obj.x0 = sim.x0(:,:,ID);
                    obj.x = zeros(sim.nx,length(sim.t));
                    obj.x(:,1) = obj.x0;
                else
                    error('Initial state must be of length n');
                end
                if(size(sim.A) == size(sim.Ahat(:,:,1)))
                    obj.A = sim.A;
                    obj.Ahat = sim.Ahat;
                    obj.xhat = zeros(sim.nx,length(sim.t),sim.N);
                    obj.xhat(:,1,:) = sim.x0;
                else
                    error('Process and prediction models must be the same size')
                end
            else
                error('Inputs must be numeric matrices')
            end
            if(strcmp(sim.PROB_TYPE,'CONTROL'))
                obj.B = sim.B;
                obj.e = zeros(sim.nx,length(sim.t));
                if(strcmp(sim.SIM_TYPE,'AV'))
                    obj.lane = obj.getLane(sim.posy(ID,1),sim.lwidth);
                    obj.pLane = ID - (obj.lane-1)*sim.Nl(obj.lane);
                    obj.a = sim.a(ID,:);
                    obj.v = sim.v(ID,:);
                    obj.px = sim.posx(ID,:);
                    obj.py = sim.posy(ID,:);
                    obj.d = sim.d(ID,:);
                    obj.dr = sim.dr(ID,:);
                    obj.se = sim.e(ID,:);
                    obj.ad = sim.u(ID,:);
                    
                    obj.Bff = sim.Bff;
                    obj.F = sim.K;
                    obj.chi = zeros(sim.nu,length(sim.t));                 
         
%                     Q = 0.1*eye(sim.nx);
%                     R = 0.01*eye(sim.nu);
%                     Qn = diag([sim.var2 sim.var3 sim.var2 sim.var3]);                    
% 
%                     Rn = zeros(sim.nx);
%                     sys = ss(obj.A,obj.B,eye(sim.nx),zeros(sim.nx,1),sim.dt);
%                     QXU = blkdiag(Q,R);
%                     QWV = blkdiag(Qn,Rn);
%                     Klqg = lqg(sys,QXU,QWV);
%                     
%                     obj.F = Klqg.C;
                    obj.psent = 0;
                    if(mod(ID,sim.Nl(obj.lane)) == 1)
                        obj.adtilde(1) = sim.uref(obj.lane,1);
                        obj.chi(:,1) = obj.F(1:3)*obj.xhat(1:3,1,ID) + obj.adtilde(1);
                    else
                        obj.adtilde(1) = sim.uref(obj.lane,1);
                        obj.chi(:,1) = obj.F(1:3)*obj.xhat(1:3,1,ID) + obj.adtilde(1);
                    end
                    
                elseif(strcmp(sim.SIM_TYPE,'STOCHASTIC'))
                    % compute controller gain
                    obj.u = zeros(sim.nu,length(sim.t));            
                    [obj.F,~,~] = dlqr(obj.A,obj.B,sim.Q,sim.R);
                    obj.u(:,1) = -obj.F*obj.x(:,1);
                end
            end
            obj.trig_type = sim.TRIG_TYPE;
            obj.z = zeros(sim.nx,length(sim.t));
            obj.z_curr = obj.z(:,1);           
            obj.ID = ID;
            obj.comm_slot = zeros(1,length(sim.t));
            obj.comm_trig = zeros(1,length(sim.t));
            obj.comm_agents = zeros(1,sim.N);
            obj.sent_state = zeros(sim.nx,1);
            obj.all_sent_states = zeros(sim.nx,sim.N);
            if(strcmp(sim.TRIG_TYPE,'ST1')||strcmp(sim.TRIG_TYPE,'ST2'))
                obj.pot_comm_slot = zeros(1,length(sim.t));
            end
            if(strcmp(sim.TRIG_TYPE,'PT') || strcmp(sim.TRIG_TYPE,'PPT'))
                obj.comm_prob = zeros(1,length(sim.t));
            end
        end
    end

    
    % run agent dynamics for one timestep
    function process(obj,sim, net, k)
%         if(nargin < 5 && net.arbit_type == 2)
%             c = 0.75;
%         end

        if(strcmp(sim.PROB_TYPE,'ESTIMATION'))
            % update state
            obj.x(:,k) = obj.A*obj.x(:,k-1) + sim.eps(:,obj.ID);

            
        elseif(strcmp(sim.PROB_TYPE,'CONTROL'))          
            if(strcmp(sim.SIM_TYPE,'AV'))
                % update state
                if(mod(obj.ID,sim.Nl(obj.lane)) == 1)
                    obj.x(:,k) = obj.A*obj.x(:,k-1) + obj.B*obj.chi(:,k-1) + obj.Bff*sim.uref(obj.lane,k-1) + sim.eps(:,obj.ID);
                    % using LQG
%                     obj.x(:,k) = obj.A*obj.x(:,k-1) + obj.B*obj.chi(:,k-1) + (obj.B + obj.Bff)*sim.uref(obj.lane,k-1) + sim.eps(:,obj.ID);

                    obj.se(k) = obj.x(1,k);
                    obj.ad(k) = obj.x(4,k);
                    
                    % update vehicle pos,vel,acc,...
                    obj.a(k) = (1-(sim.dt/sim.tau))*obj.a(k-1) + (sim.dt/sim.tau)*obj.ad(k-1);
                    obj.v(k) = obj.v(k-1) + sim.dt*obj.a(k-1);
                    obj.px(k) = obj.px(k-1) + sim.dt*obj.v(k-1);           
                    
                else % ID ~= 1
                    obj.x(:,k) = obj.A*obj.x(:,k-1) + obj.B*obj.chi(:,k-1) + obj.Bff*obj.xhat(4,k-1,obj.ID-1) + sim.eps(:,obj.ID);
                    % using LQG
%                     obj.x(:,k) = obj.A*obj.x(:,k-1) + obj.B*obj.chi(:,k-1) + (obj.B + obj.Bff)*obj.xhat(4,k-1,obj.ID-1) + sim.eps(:,obj.ID);
                    
                    obj.se(k) = obj.x(1,k);
                    obj.ad(k) = obj.x(4,k);

                    % update vehicle pos,vel,acc,...
                    obj.a(k) = (1-(sim.dt/sim.tau))*obj.a(k-1) + (sim.dt/sim.tau)*obj.ad(k-1);
                    obj.v(k) = obj.v(k-1) + sim.dt*obj.a(k-1);
                    obj.px(k) = obj.px(k-1) + sim.dt*obj.v(k-1);           
                    
                end
                
%                 if(strcmp(sim.TRIG_TYPE,'ET'))
%                     obj.tau(k) = obj.timeSinceLastTrig(k,sim.dt);                  
%                     obj.eta(k) = obj.eta(k) + sim.dt*obj.psi(k-1);
%                 end
  
                
            elseif(strcmp(sim.SIM_TYPE,'STOCHASTIC'))
                
                obj.x(:,k) = obj.A*obj.x(:,k-1) + obj.B*obj.u(:,k-1) + sim.eps(:,obj.ID);

            end  
        end
        
        % Communicate
        if(strcmp(sim.TRIG_TYPE,'ET'))           
            obj.ET_comm(sim,k);
            
        elseif(strcmp(sim.TRIG_TYPE,'PT') || strcmp(sim.TRIG_TYPE,'PPT'))
            obj.PT_comm(sim,net,k);
            
        elseif(strcmp(sim.TRIG_TYPE,'ST1') || strcmp(sim.TRIG_TYPE,'ST2'))
            obj.ST1_comm(sim,sim.ST_M,k);
            
        end 
    end
    
    
    % set agent predictions based on which agents have communicated their
    % states
    function predictors(obj,sim,k)
        
        % update all predicted states (deterministic model)
        if(strcmp(sim.PROB_TYPE,'ESTIMATION'))
            if(obj.comm_trig(k))
                obj.xhat(:,k,obj.ID) = obj.x(:,k);
                for id = 1:size(obj.Ahat,3)
                    if(id ~= obj.ID)
                        obj.xhat(:,k,id) = obj.Ahat(:,:,id)*obj.xhat(:,k-1,id);
                    end
                end
            else
                for id = 1:size(obj.Ahat,3)
                    obj.xhat(:,k,id) = obj.Ahat(:,:,id)*obj.xhat(:,k-1,id);
                end             
            end
   
        elseif(strcmp(sim.PROB_TYPE,'CONTROL')) 
            if(strcmp(sim.SIM_TYPE,'AV'))
                          
                for id = 1:size(obj.Ahat,3)
                    if(mod(id,sim.Nl(obj.lane)) == 1)
                        if(obj.comm_agents(id)==0)
                            obj.xhat(:,k,id) = obj.Ahat(:,:,id)*obj.xhat(:,k-1,id) + obj.B*obj.chi(:,k-1) + obj.Bff*sim.uref(obj.lane,k-1);
%                             obj.xhat(:,k,id) = obj.Ahat(:,:,id)*obj.xhat(:,k-1,id) + obj.B*obj.chi(:,k-1) + (obj.B + obj.Bff)*sim.uref(obj.lane,k-1);
                        
                        elseif(obj.comm_agents(id)==1)
                            xcomm = obj.all_sent_states(:,id);
                            obj.xhat(:,k,id) = xcomm;
                        
                        end
                      
                    else
                        
                        if(obj.comm_agents(id)==0)
                            obj.xhat(:,k,id) = obj.Ahat(:,:,id)*obj.xhat(:,k-1,id) + obj.B*obj.chi(:,k-1) + obj.Bff*obj.xhat(4,k-1,id-1);                        
%                             obj.xhat(:,k,id) = obj.Ahat(:,:,id)*obj.xhat(:,k-1,id) + obj.B*obj.chi(:,k-1) + (obj.B + obj.Bff)*obj.xhat(4,k-1,id-1);                        
                            
                        elseif(obj.comm_agents(id)==1)
                            xcomm = obj.all_sent_states(:,id);
                            obj.xhat(:,k,id) = xcomm;
                            
                        end
                    end
                end

 
            elseif(strcmp(sim.SIM_TYPE,'STOCHASTIC'))

                % update all predicted states (perfect model)
                if(obj.comm_trig(k))
                    obj.xhat(:,k,obj.ID) = obj.x(:,k);
                    for id = 1:size(obj.Ahat,3)
                        if(id ~= obj.ID)
                            obj.xhat(:,k,id) = obj.Ahat(:,:,id)*obj.xhat(:,k-1,id) + obj.B*obj.u(:,k-1);
                        end
                    end
                else
                    for id = 1:size(obj.Ahat,3)
                        obj.xhat(:,k,id) = obj.Ahat(:,:,id)*obj.xhat(:,k-1,id) + obj.B*obj.u(:,k-1);
                    end               
                end
                
            end       
        end
    end
    
    
    % calculate error Z of current agent
    function error(obj,k)
        obj.z(:,k) = obj.x(:,k) - obj.xhat(:,k,obj.ID);
        obj.z_curr = obj.z(:,k);
        
    end
    
    
    % update control input
    function controller(obj,sim,k)
        if(strcmp(sim.SIM_TYPE,'AV'))           
            if(strcmp(obj.trig_type,'ET'))
                % ZOH method for ET
%                 if(mod(obj.ID,sim.Nl(obj.lane)) == 1)
%                     obj.chi(:,k) = obj.F(1:3)*obj.x(1:3,k) + sim.uref(obj.lane,k);
%                 else
%                     spacing_error = obj.ppreced - obj.px(k) - sim.L(obj.ID) - sim.r(obj.ID) - sim.h*obj.v(k);
% 
%                     if(obj.comm_agents(obj.ID-1)==0)
%                         obj.adtilde(k) = obj.adtilde(k-1);
%                     elseif(obj.comm_agents(obj.ID-1)==1)
%                         obj.adtilde(k) = obj.all_sent_states(4,obj.ID-1);
%                     end
%                     obj.chi(:,k) = obj.F(1:3)*[spacing_error;obj.x(2:3,k)] + obj.adtilde(k);
% 
%                 end

                % model-prediction for ET
                if(mod(obj.ID,sim.Nl(obj.lane)) == 1)
                    obj.chi(:,k) = obj.F(1:3)*obj.xhat(1:3,k,obj.ID) + sim.uref(obj.lane,k);
                else
                    spacing_error = obj.ppreced - obj.px(k) - sim.L(obj.ID) - sim.r(obj.ID) - sim.h*obj.v(k);
                    obj.chi(:,k) = obj.F(1:3)*[spacing_error;obj.xhat(2:3,k,obj.ID)] + obj.xhat(4,k,obj.ID-1);
%                     obj.chi(:,k) = obj.F(1:3)*obj.xhat(1:3,k,obj.ID) + obj.xhat(4,k,obj.ID-1);

                end
                
            else
                
                if(mod(obj.ID,sim.Nl(obj.lane)) == 1)
                    obj.chi(:,k) = obj.F(1:3)*obj.xhat(1:3,k,obj.ID) + sim.uref(obj.lane,k);
                else
                    spacing_error = obj.ppreced - obj.px(k) - sim.L(obj.ID) - sim.r(obj.ID) - sim.h*obj.v(k);
                    obj.chi(:,k) = obj.F(1:3)*[spacing_error;obj.xhat(2:3,k,obj.ID)] + obj.xhat(4,k,obj.ID-1);
%                     obj.chi(:,k) = obj.F(1:3)*obj.xhat(1:3,k,obj.ID) + obj.xhat(4,k,obj.ID-1);

                end
                
            end
            
        elseif(strcmp(sim.SIM_TYPE,'STOCHASTIC'))
            obj.u(:,k) = -obj.F*obj.xhat(:,k,obj.ID); 
            
        end  
    end

    
    % determine comm probability at time k+M taking horizon into account
    function pred_comm_prob(obj,sim,k)
        if(k <= length(sim.t) - sim.M)
            lookup = [sim.reset_prob; obj.H'];
            Pc = obj.comm_prob(k);
            numTerms = 2^sim.M;
            numSubTerms = sim.M+1;
            c = dec2bin(0:numTerms-1)-48;
            c = rot90(c,2);
            prob = 0;
            for n = 1:numTerms   
                if(sum(c(n,:)) == sim.M)
                    term = Pc*lookup(1,1)^sim.M;
                elseif(sum(c(n,:)) == 0)
                    term = 1;
                    for m = 1:sim.M
                        if(m == sim.M)
                            term = term*lookup(m);
                        else
                            term = term*(1-lookup(m));
                        end
                    end
                    term = term*(1-Pc);
                else

                    term = 1;       
                    if(c(n,1))
                        term = term*Pc;
                    else
                        term = term*(1-Pc);
                    end

                    % c(s) determines 1-H or just H
                    for s = 1:numSubTerms-1 
                        Z = 2;          % set to error not zero row
                        t = 0;
                        for m = s:-1:1    
                            if(c(n,m))
                                Z = 1;          % set to reset probability row
                                t = s-m+1;
                                break;
                            else
                                t = t+1;        % increment timestep index
                            end
                        end

                        if(s+1 <= numSubTerms-1)
                            if(c(n,s+1))
                                term = term*lookup(Z,t);
                            else
                                term = term*(1-lookup(Z,t));
                            end
                        else
                            term = term*lookup(Z,t);
                        end
                    end 

                end

                prob = prob + term;
            end
            
            obj.comm_prob(1,k+sim.M) = prob;
            obj.comm_prob_mstep = prob;
        end
    end


    % ARBITRATOR: Decides communication at current time
    function PT_comm(obj,sim,net,k,c)
        if(nargin < 5 && net.arbit_type == 2)
            c = 0.75;
        end
        
        obj.sent_state = zeros(sim.nx,1);
        
        % comm if assigned slot
        if(net.arbit_type == 0)
            if(obj.comm_slot(k))
%                 obj.xhat(:,k,obj.ID) = obj.x(:,k);
                obj.comm_trig(k) = obj.comm_slot(k);
                obj.sent_state = obj.x(:,k);
            end
            
        % comm if assigned slot and error > delta
        elseif(net.arbit_type == 1)
            if(obj.comm_slot(k))
                % if error really is above threshold, communicate
                if(norm(obj.z(:,k-1)) >= sim.delta)
                    obj.comm_trig(k) = 1;
                    obj.sent_state = obj.x(:,k);
                end
            end

        % comm if assigned slot and error > c*delta
        elseif(net.arbit_type == 2)
            if(c > 1)
                error('Input a constant factor in (0,1)');
            end
            if(obj.comm_slot(k))
                if(norm(obj.z(:,k-1)) >= c*sim.delta)
                    obj.comm_trig(k) = 1;
                    obj.sent_state = obj.x(:,k);          
                end                
            end    
        end
        
        obj.comm_trig_curr = obj.comm_trig(k);
        
        if(strcmp(sim.SIM_TYPE,'AV'))
            obj.psent = obj.px(k);
        end
    end
    
    function schedule_slot(obj,sim,k)
        % set slot assignment M step ahead
        if(k <= length(sim.t) - sim.M)
            obj.comm_slot(k+sim.M) = obj.comm_slot_mstep;
        end
    end
    
    
    % EVENT-TRIGGERED COMMUNICATION [no prediction]
    function ET_comm(obj,sim,k)
        obj.sent_state = zeros(sim.nx,1);
        
        % communicate at every timestep
%         if(obj.comm_slot(k))
%             obj.comm_trig(k) = 1;
%             obj.sent_state = obj.x(:,k-1);
%             obj.comm_trig_curr = obj.comm_trig(k);
%         end
        
        if(obj.comm_slot(k))
            if(norm(obj.z(:,k-1)) >= sim.delta)
                obj.comm_trig(k) = 1;
                obj.sent_state = obj.x(:,k);
            end
        end
        obj.comm_trig_curr = obj.comm_trig(k);
        
        if(strcmp(sim.SIM_TYPE,'AV'))
            obj.psent = obj.px(k);
        end
    end
    
%     function ET_Heemels(obj,sim,k)
%         obj.sent_state = zeros(sim.nx,1);
%         
%         if(obj.comm_slot(k))
%             if(obj.eta(k-1) < 0)
%                 obj.comm_trig(k) = 1;
%                 obj.sent_state = obj.x(:,k);
%             end
%         end
%         obj.comm_trig_curr = obj.comm_trig(k);
%         
%         obj.psent = obj.px(k);
%     end
    
    
    % SELF-TRIGGERED COMMUNICATION
    function ST1_comm(obj,sim,ST_M,k)
        
        obj.sent_state = zeros(sim.nx,1);
        
        % communicate if slot assigned
        if(obj.comm_slot(k) == 1)
            % arbitration
            obj.xhat(:,k,obj.ID) = obj.x(:,k);
            obj.comm_trig(k) = obj.comm_slot(k);  
            obj.sent_state = obj.x(:,k);

            % reset agent to ST
            if(~strcmp(obj.trig_type,'ST1'))
                obj.trig_type = 'ST1';
                obj.M = ST_M;                
                obj.comm_slot(k+1:k+obj.M-1) = 0;
            end
            
            % compute new M
            if(size(obj.A,3) ~= 1)      % time-varying agent model                            
            end

        elseif(obj.comm_slot(k) == -1)
            obj.trig_type = 'PT';
            obj.M = sim.M;
        end
        
        obj.comm_trig_curr = obj.comm_trig(k);        
    end
    
    function ST2_comm(obj,t,M,k)
        
        obj.sent_state = zeros(sim.nx,1);
        
        % communicate if slot assigned
        if(obj.comm_slot(k))
            % arbitration
            obj.xhat(:,k,obj.ID) = obj.x(:,k);
            obj.comm_trig(k) = obj.comm_slot(k);  
            obj.sent_state = obj.x(:,k);
        
            % reset agent to ST
            if(~strcmp(obj.trig_type,'ST2'))
                obj.trig_type = 'ST2';
                obj.M = M;                
                obj.comm_slot(k+1:k+obj.M-1) = 0;
            end
            
            % compute new M
            if(size(obj.A,3) ~= 1)      % time-varying agent model                            
            end

            % set next potential comm slot
            if(k+obj.M <= length(t))
                obj.pot_comm_slot(k+obj.M) = 1;
            end
        end
        
        obj.comm_trig_curr = obj.comm_trig(k);      
    end
    
    
    % Return number of errors above threshold
    function count = check_error(obj,sim)
        norm_z = obj.get_estError_norm(sim.nx);

        count = 0;
        for k = 1:length(sim.t)
            if(norm_z(k) >= sim.delta)
                
                if(k + sim.M <= length(sim.t))
                    if(norm_z(k+sim.M) >= sim.delta)
                        count = count + 1;
                    end
                end
                % consider all M prediction points
%                     for m = sim.M:-1:1
%                         if(norm_z(k+m) >= sim.delta)
% 
%                         end
%                     end
            end
        end
        % number of points above threshold
%             count = numel(norm_z(norm_z >= delta));
    end

    
    % get error norm of agent for over entire simulation
    function znorm = get_estError_norm(obj,nx)
        if(nx > 1)
            znorm = sqrt(sum(obj.z.*obj.z));
        else
            znorm = abs(obj.z);
        end
    end
    
    % returns mean estimation error norm of a simulation for this agent
    function zbari = mean_estError_norm(obj,nx)
        norm_z = obj.get_estError_norm(nx);
        zbari = mean(norm_z);
    end 
    
    
    % get error norm of agent for over entire simulation
    function enorm = get_ctrlError_norm(obj,sim)
        if(strcmp(sim.SIM_TYPE,'STOCHASTIC'))
            xd = zeros(sim.nx,length(sim.t));
            obj.e = obj.x - xd;
            
            if(sim.nx > 1)
                enorm = sqrt(sum(obj.e.*obj.e));
            else
                enorm = abs(obj.e);
            end
              
        elseif(strcmp(sim.SIM_TYPE,'AV'))
            if(mod(obj.ID,sim.Nl(obj.lane)) == 1)
                obj.e = abs(obj.v - sim.vref(obj.lane,:));
                enorm = obj.e;
            else
                obj.e = [obj.v;obj.d]-[sim.vref(obj.lane,:);obj.dr];
                enorm = sqrt(sum(obj.e.*obj.e));                
            end
        end
    end
    
    % returns mean control error norm of a simulation for this agent
    function ebari = mean_ctrlError_norm(obj,sim)
        norm_e = obj.get_ctrlError_norm(sim);
        ebari = mean(norm_e);
    end
    

    function lane = getLane(~,ypos,lwidth)
        lane = 1 + (ypos - lwidth/2)/lwidth;
        lane = int8(lane);
    end
    
    function lastTrig = timeSinceLastTrig(obj,k,dt)
        commTimes = find(obj.comm_trig == 1);
        if(isempty(commTimes))
            lastTrig = k*dt;
        else
            lastComm = commTimes(end);
            lastTrig = (k-lastComm)*dt;
        end
    end
    
    % Plot the state trajectories and the error of the agent over time
    function plotStates(obj,nx,t,delta)
        figure;
        for i = 1:nx
            subplot(nx+1,1,i);
            hold on; grid on; box on; 
            plot(t,obj.x(i,:));
            plot(t,obj.xhat(i,:,obj.ID));
            xlabel('Time [s]');
            ylabel(['x',num2str(i)]); 
            if(i == 1)
                title(['Agent ',num2str(obj.ID)]);
                legend('Process','Prediction');
            end
        end
        subplot(nx+1,1,nx+1);
        hold on; grid on; box on; 
        norm_z = obj.get_estError_norm(nx);
        plot(t,norm_z);
        plot(t,delta*ones(1,length(t)),'r');
        xlabel('Time [s]');
        ylabel('||Z(t)||_{2}'); 
    end

    function plotEstError(obj,nx,t,delta)
        figure;
        subplot(2,1,1)
        hold on; grid on; box on; 
        norm_z = obj.get_estError_norm(nx);
        plot(t,norm_z);
        plot(t,delta*ones(1,length(t)),'r');
        title(['Agent ',num2str(obj.ID)]);
        xlabel('Time [s]');
        ylabel('||Z(t)||_{2}');
        legend('State Error','Error Bound');
        subplot(2,1,2);
        hold on; grid on; box on;         
        stairs(t,0.5*obj.comm_slot(1:length(t)),'g');
        stairs(t,obj.comm_trig,'k');
        legend('Comm Slot','Comm Trigger');
        xlabel('Time [s]');
        ylim([0 1.5]);
    end
    
    function plotCtrlError(obj,sim)
        figure;
        subplot(2,1,1)
        hold on; grid on; box on; 
        norm_e = obj.get_ctrlError_norm(sim);
        plot(sim.t,norm_e);
        title(['Agent ',num2str(obj.ID)]);
        xlabel('Time [s]');
        ylabel('||e(t)||_{2}');
        legend('Control Error');
        subplot(2,1,2);
        hold on; grid on; box on;         
        stairs(sim.t,0.5*obj.comm_slot(1:length(sim.t)),'g');
        stairs(sim.t,obj.comm_trig,'k');
        legend('Comm Slot','Comm Trigger');
        xlabel('Time [s]');
        ylim([0 1.5]);
    end 

end
end