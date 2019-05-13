function [slots,tie] = NetworkManager(varargin)
% Scheduler: network slot assignment for time k+M
%
%   sim     structure containing the simulation parameters
%   net     structure containing the network parameters
%   H       Mstep comm. prob. of all agents
%
% Outputs:
%   slots   slots allocation for all agents (binary {0,1})   
%   ties    if a critical tie occurs at the current time step
%           (critical is if tie occurs at the Kth slot) 
%--------------------------------------------------------------------------


if(nargin < 5)
    if(nargin < 4)
        constraint = 0;
        if(nargin < 3)
            error('Not enough input arguments');
        end
    else
        % PT w/ lower bound constraint
        constraint = 1;
        ids = varargin{4};
    end
else
    % ST
    ids = varargin{4};
    NAS = varargin{5};
end

sim = varargin{1};
net = varargin{2};
H = varargin{3};
trig_type = sim.TRIG_TYPE;
tie = 0;


if(strcmp(trig_type,'PT') || strcmp(trig_type,'PPT'))
    if(~constraint)
        %------------------------------------------------------------------
        % METHOD 1 - PT or PPT (with no constraint on which probs sent):
        %            K largest communication probabilities probabilities
        %------------------------------------------------------------------
        if(net.sched_type == 0)
            if(sim.N <= net.K)
                slots = ones(1,sim.N);

            else
                slots = zeros(1,sim.N);

                % sort based on hitting probabilities
                [Hs,ID] = sort(H,'descend');
                [C,~,IC] = unique(Hs,'stable');

                % if ties occur, randomize 
                if(length(Hs) ~= length(C))                
                    if(Hs(net.K) == Hs(net.K+1))
                        n1 = numel(find(IC(1:net.K)==IC(net.K)));
                        n2 = numel(find(IC==IC(net.K)));
                        a = ID(net.K-n1+1:net.K+n2-n1);
                        a = a(randperm(n2));
                        ID(net.K-n1+1:net.K+n2-n1) = a;
                        tie = tie + 1;
                    end
                end

                for i = 1:net.K
                    slots(ID(i)) = 1;
                end
            end

        %------------------------------------------------------------------
        % METHOD 2 - PT or PPT (with no constraint on which probs sent):
        %            Assign slots to agents w/ prob > h
        %------------------------------------------------------------------
        elseif(net.sched_type == 1)
            slots = zeros(1,sim.N);
            setI = find(H > net.h);

            % more agents have prob > h
            if(length(setI) > net.K)
                % sort based on hitting probabilities
                [Hs,ID] = sort(H,'descend');
                [C,~,IC] = unique(Hs,'stable');

                % if ties occur, randomize 
                if(length(Hs) ~= length(C))                
                    if(Hs(net.K) == Hs(net.K+1))
                        n1 = numel(find(IC(1:net.K)==IC(net.K)));
                        n2 = numel(find(IC==IC(net.K)));
                        a = ID(net.K-n1+1:net.K+n2-n1);
                        a = a(randperm(n2));
                        ID(net.K-n1+1:net.K+n2-n1) = a;
                        tie = tie + 1;
                    end
                end
                for i = 1:net.K
                    slots(ID(i)) = 1;
                end
                % random set of agents > h
        %         b = sort(randsample(setI,network.K,0));
        %         slots(b) = 1;
            else
                slots = double((H > net.h));
            end
        end  
        
    else
        %------------------------------------------------------------------
        % METHOD 1 - PT or PPT (with constraint prob > h/d):
        %            K largest communication probabilities probabilities
        %------------------------------------------------------------------
        if(net.sched_type == 0)           
            slots = zeros(1,sim.N);
            if(length(ids) <= net.K)
                if(~isempty(ids))
                    slots(ids) = 1;
                end

            else
                % sort based on hitting probabilities
                [Hs,ID] = sort(H,'descend');
                ids = ids(ID);
                [C,~,IC] = unique(Hs,'stable');

                % if ties occur, randomize 
                if(length(Hs) ~= length(C))                
                    if(Hs(net.K) == Hs(net.K+1))
                        n1 = numel(find(IC(1:net.K)==IC(net.K)));
                        n2 = numel(find(IC==IC(net.K)));
                        a = ids(net.K-n1+1:net.K+n2-n1);
                        a = a(randperm(n2));
                        ids(net.K-n1+1:net.K+n2-n1) = a;
                        tie = tie + 1;
            %                 ties = length(H)-length(C);
                    end
                end

                for i = 1:net.K
                    slots(ids(i)) = 1;
                end
            end
            
        %------------------------------------------------------------------
        % METHOD 2 - PT or PPT (with no constraint on which probs sent):
        %            Assign slots to agents w/ prob > h
        %------------------------------------------------------------------
        elseif(net.sched_type == 1)
            slots = zeros(1,sim.N);
            setI = find(H > net.h);

            % more agents have prob > h
            if(length(setI) > net.K)
                % sort based on hitting probabilities
                [Hs,ID] = sort(H,'descend');
                ids = ids(ID);
                [C,~,IC] = unique(Hs,'stable');

                % if ties occur, randomize 
                if(length(Hs) ~= length(C))                
                    if(Hs(net.K) == Hs(net.K+1))
                        n1 = numel(find(IC(1:net.K)==IC(net.K)));
                        n2 = numel(find(IC==IC(net.K)));
                        a = ids(net.K-n1+1:net.K+n2-n1);
                        a = a(randperm(n2));
                        ids(net.K-n1+1:net.K+n2-n1) = a;
                        tie = tie + 1;
                    end
                end
                for i = 1:net.K
                    slots(ids(i)) = 1;
                end
                
            else
                slots(ids(setI)) = 1;
            end
        end
        
    end
    

elseif(strcmp(trig_type,'ST'))
% in this case H represents the M values sent to NetMan
% priority goes first to PT agents

    %split into probs and Ms
    Hm = []; Hp = []; IDm = []; IDp = [];
    for n = 1:length(H)
        if(isreal(H(n)) && rem(H(n),1)==0 && H(n)~=1)
            Hm = [Hm H(n)];
            IDm = [IDm ids(n)];
        else
            Hp = [Hp H(n)];
            IDp = [IDp ids(n)];
        end
    end
    
    maxM = max(Hm);
    slots = zeros(sim.N,maxM);

    % assign PT agents
    if(net.sched_type == 0)
        if(length(Hp) > net.K)
            [Hs,id] = sort(Hp,'descend');
            IDp = IDp(id);
            [C,~,IC] = unique(Hs,'stable');
            
            % if ties occur, randomize 
            if(length(Hs) ~= length(C))                
                if(Hs(net.K) == Hs(net.K+1))
                    n1 = numel(find(IC(1:net.K)==IC(net.K)));
                    n2 = numel(find(IC==IC(net.K)));
                    a = IDp(net.K-n1+1:net.K+n2-n1);
                    a = a(randperm(n2));
                    IDp(net.K-n1+1:net.K+n2-n1) = a;
                    tie = tie + 1;
                end
            end
            for k = 1:net.K
                slots(IDp(k),sim.M) = 1;
            end
        else
            slots(IDp,sim.M) = 1;
        end
        
    elseif(net.sched_type == 1)
        setI = find(Hp > net.h);

        % more agents have prob > h
        if(length(setI) > net.K)
            [Hs,id] = sort(Hp,'descend');
            IDp = IDp(id);
            [C,~,IC] = unique(Hs,'stable');

            % if ties occur, randomize 
            if(length(Hs) ~= length(C))                
                if(Hs(net.K) == Hs(net.K+1))
                    n1 = numel(find(IC(1:net.K)==IC(net.K)));
                    n2 = numel(find(IC==IC(net.K)));
                    a = IDp(net.K-n1+1:net.K+n2-n1);
                    a = a(randperm(n2));
                    IDp(net.K-n1+1:net.K+n2-n1) = a;
                    tie = tie + 1;
                end
            end
            for k = 1:net.K
                slots(IDp(k),sim.M) = 1;
            end

        else
            slots(IDp(setI),sim.M) = 1;
        end
    end
    
    % assign ST agents
    count = zeros(1,maxM);
    for m = 1:maxM
        mpt = 0;
        % time step at prediction horizon of PT agents
        if(m == sim.M)
            mpt = sum(slots(:,m));
            numAvailable = net.K - mpt;
        else
            numAvailable = net.K;
        end
        IDs = [];
        for h = 1:length(Hm)
            if(Hm(h) == m)
                count(m) = count(m)+1;
                IDs = [IDs IDm(h)];
            end
        end
        if(count(m) > numAvailable)
            I = randperm(length(IDs),numAvailable);
            i = IDs(I);
            slots(i,m) = 1;
            % set flag to switch to PT or recompute M
            IDs(I) = [];
            slots(IDs,m) = -1;
        else
            if(~isempty(IDs))
                slots(IDs,m) = 1;
            end
        end
    end

%     for m = 1:maxM
%         IDs = [];
%         for h = 1:length(H)
%             if(H(h) == m)
%                 count(m) = count(m)+1;
%                 IDs = [IDs ids(h)];
%             end
%         end
%         if(count(m) > net.K-prevDiff)
%             diff = count(m) - (net.K-prevDiff);
%             if(diff >= count(m))
%                 
%             else
%                 I = randperm(length(IDs),net.K-prevDiff);
%                 i = IDs(I);
%                 slots(i,m) = 1;
%                 IDs(I) = [];
%                 slots(IDs,m+1) = 1;
%             end
% 
%         else
%             diff = 0;
%             if(~isempty(IDs))
%                 slots(IDs,m) = 1;
%             end
%         end
%         prevDiff = diff;
%     end
    
elseif(strcmp(trig_type,'ST2'))
    %----------------------------------------------------------------------
    % METHOD 1 - ST: K largest D-step probabilities
    %----------------------------------------------------------------------
    if(net.sched_type == 0)           
        if(sim.N <= net.K)
            slots = ones(sim.N,1);
        else
            slots = zeros(sim.N,ceil((length(H)-(net.K-NAS))/net.K)+1);
            % sort based on comm probs
            [Hs,ID] = sort(H,'descend');
            ids = ids(ID);
            for m = 1:size(slots,2)
                if(m == 1)
                    slotsAvailable = net.K-NAS;
                else
                    slotsAvailable = net.K;
                end
                if(length(Hs) <= slotsAvailable)
                    for i = 1:length(Hs)
                        slots(ids(i),m) = 1;
                    end
                else
                    [C,~,IC] = unique(Hs,'stable');

                    % if ties occur, randomize 
                    if(length(Hs) ~= length(C))             
                        if(Hs(net.K-NAS) == Hs(net.K-NAS+1))
                            n1 = numel(find(IC(1:net.K-NAS)==IC(net.K-NAS)));
                            n2 = numel(find(IC==IC(net.K-NAS)));
                            a = ids(net.K-NAS-n1+1:net.K-NAS+n2-n1);
                            a = a(randperm(n2));
                            ids(net.K-NAS-n1+1:net.K-NAS+n2-n1) = a;
                            tie = tie + 1;
                %                 ties = length(H)-length(C);
                        end
                    end
                    if(m == 1)
                        for i = 1:net.K-NAS
                            slots(ids(i),m) = 1;
                        end
                        Hs(1:net.K-NAS) = [];
                        ids(1:net.K-NAS) = [];
                    else
                        for i = 1:net.K
                            slots(ids(i),m) = 1;
                        end
                        Hs(1:net.K) = [];
                        ids(1:net.K) = [];
                    end
                end
            end
        end

    %----------------------------------------------------------------------
    % METHOD 2 - ST: Agents w/ prob > h
    %----------------------------------------------------------------------
    elseif(net.sched_type == 1)
        IDs = ids;
        slots = zeros(sim.N,ceil((length(H)-(net.K-NAS))/net.K)+1);
        IDg = H > net.h;
        IDl = H <= net.h;
        Hg = H(IDg);

        % more agents have prob > h
        if(length(Hg) > net.K)
            % sort based on hitting probabilities
            [Hs,ID] = sort(H,'descend');
            ids = ids(ID);
            for m = 1:size(slots,2)
                if(m == 1)
                    slotsAvailable = net.K-NAS;
                else
                    slotsAvailable = net.K;
                end
                if(length(Hs) <= slotsAvailable)
                    for i = 1:length(Hs)
                        if(Hs(i) > net.h)
                            slots(ids(i),m) = 1;
                        end
                    end
                else
                    [C,~,IC] = unique(Hs,'stable');

                    % if ties occur, randomize 
                    if(length(Hs) ~= length(C))             
                        if(Hs(net.K-NAS) == Hs(net.K-NAS+1))
                            n1 = numel(find(IC(1:net.K-NAS)==IC(net.K-NAS)));
                            n2 = numel(find(IC==IC(net.K-NAS)));
                            a = ids(net.K-NAS-n1+1:net.K-NAS+n2-n1);
                            a = a(randperm(n2));
                            ids(net.K-NAS-n1+1:net.K-NAS+n2-n1) = a;
                            tie = tie + 1;
                %                 ties = length(H)-length(C);
                        end
                    end
                    if(m == 1)
                        for i = 1:net.K-NAS
                            if(Hs(i) > net.h)
                                slots(ids(i),m) = 1;
                            end
                        end
                        Hs(1:net.K-NAS) = [];
                        ids(1:net.K-NAS) = [];
                    else
                        for i = 1:net.K
                            if(Hs(i) > net.h)
                                slots(ids(i),m) = 1;
                            end
                        end
                        Hs(1:net.K) = [];
                        ids(1:net.K) = [];
                    end
                end
            end
            % assign agents w/ prob <=h to switch to PT
            slots(IDs(IDl),1) = -1;

        else
            for i = 1:length(H)
                if(H(i) > net.h)
                    slots(ids(i),1) = 1;
                else
                    % assign agents w/ prob <=h to switch to PT
                    slots(ids(i),1) = -1;
                end
            end
        end   
    end
end