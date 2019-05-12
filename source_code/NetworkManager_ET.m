function [slots] = NetworkManager_ET(sim,net,znorms)
% Scheduler: network slot assignment for time k+M using the Mamduhi paper
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

P = zeros(1,sim.N);
slots = zeros(1,sim.N);
lambda = 0;                     % scheduler bound
ids = linspace(1,sim.N,sim.N);
znorms = znorms.^2;

% priority measure
znorms_afterbound = znorms(znorms > lambda);
ids_afterbound = find(znorms>lambda);
jk = length(ids_afterbound);

for ID = 1:sim.N
    if(znorms(ID) <= lambda)
        P(ID) = 0;
    else
        if(jk <= net.K)
%             P(ID) = 1;
            slots(ids_afterbound) = 1;
            break;
        else
            P(ID) = znorms(ID)/sum(znorms_afterbound);
        end
    end
end

% biased randomization
if(jk > net.K)
    for k = 1:net.K
        a(k) = randsample(ids,1,true,P);
        P(ids == a(k)) = 0;
        slots(a(k)) = 1;
    end
end