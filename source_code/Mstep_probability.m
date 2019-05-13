function H = Mstep_probability(sim,trig_type,M,z)
% Function to calculate Pr(tau > M*dt | Z(0) = z) of all agents by using
% the precomputed lookup tables
% 
% Inputs:
%   sim         structure containing the simulation parameters
%   trig_type   trigger type [string]
%   M           prediction horizon
%   z           error of all agents
%
% Outputs:      
%   H           exit probability of all agents
%
%--------------------------------------------------------------------------

if(M == 0)  % ET
    znorm = sqrt(sum(z.*z));
    H = znorm >= sim.delta;
else
    if(strcmp(trig_type,'PT') || strcmp(trig_type,'PPT'))
        H = zeros(M,size(z,2));

        for m = 1:M
            for i = 1:size(z,2)
                if(norm(z(:,i)) >= sim.delta)
                    H(:,i) = ones(M,1);
                else
                    z0 = num2cell(z(:,i));
                    H(m,i) = interpn(sim.Z0_grid{:},sim.lookup_prob{m},z0{:});
                end
            end
        end

    elseif(strcmp(trig_type,'ST') || strcmp(trig_type,'ST2'))

        H = zeros(1,size(z,2));
        for i = 1:size(z,2)
            if(norm(z(:,i)) >= sim.delta)
                H(i) = 1;
            else
                z0 = num2cell(z(:,i));
                H(i) = interpn(sim.Z0_grid{:},sim.sched_delay_lookup,z0{:});
            end
        end

    end
end