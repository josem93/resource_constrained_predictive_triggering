function run_Monte_Carlo(sim, file1,file2, folder)
% Function to run a Monte Carlo simulation of the error process 
%
% sim:      structure containing simulation parameters
% file1:    filename for lookup table [string]
% file2:    filename for reset probability [string]
% folder:   save location folder name
%
%--------------------------------------------------------------------------

B = sim.numSim;

Ac = (1/sim.dt)*(sim.A - eye(sim.nx));
Qc = sqrt(sim.Q)/sqrt(sim.dt);
if(sim.INPUT_NOISE)
    Bc = sim.B/sim.dt;
end

fprintf('Monte Carlo Simulation in progress...');
tMC = tic;

% eps = Qc*randn(sim.nx,sim.M,B);
W = sqrt(sim.dt)*randn(sim.nx,sim.M,B);

prob = cell(sim.nx,1);
[prob{:}] = ndgrid(zeros(1,length(sim.Z0_set)));
prob = prob{1};

reset_prob = zeros(1,sim.M);

v = ones(1,sim.nx);         % index vector
ready = false;
reverseStr = '';
% P = zeros(size(prob),M);
% for m = 1:M
while ~ready
    V = num2cell(v);
    index = sub2ind(size(prob),V{:});
    
    z0 = sim.Z0_set(v)';
    if(norm(z0) >= sim.delta)
        prob(index) = 1;
    else
        count = 0;
        
        for b = 1:B
            z_prev = z0;
           
            %-- Stopping Times --%
            if(strcmp(sim.MC_type,'STO'))
                % run error process M steps and if its norm exceeds delta 
                % at any point increment count
                for k = 1:sim.M
                    % update error process
                    if(sim.INPUT_NOISE)
%                         z = sim.A*z_prev + sim.B*eye(sim.nx)*eps(:,k,b);
                        z = sim.A*z_prev + Qc*Bc;                         
                    else
                        z = (eye(size(Ac,1)) + Ac*sim.dt)*z_prev + Qc*W(:,k,b); 
                    end
%                     z = sim.A*z_prev + eps(:,k,b);
                    z_prev = z;
                    if(norm(z) >= sim.delta)
                        count = count + 1;
                        break;
                    end
                end
                
            %-- Error --%  [MAY NOT NEED]
            elseif(strcmp(sim.MC_type,'ERR'))
                % only checks error after M steps
                for k = 1:sim.M
                    % update stochastic error process
                    eps =  sqrt(sim.Q)*randn(sim.nx,1);
                    z = sim.A*z_prev + eps; 
                    z_prev = z;
                end
                if(norm(z) >= sim.delta)
                    count = count + 1;
                end
            end
        end
        prob(index) = count/B;
%         if(norm(z0) == 0)
%             reset_prob(sim.M) = count/B;
%         end
    end
    
    % n-for loops tracked by index vector
    ready = true;
    for m = 1:sim.nx
        v(m) = v(m) + 1;
        if(v(m) <= length(sim.Z0_set))
            ready = false;
            break;
        end
        v(m) = 1;
    end
    
    % display percentage completed
    percentDone = floor(100*index/numel(prob));
    msg = sprintf('%i', percentDone);
    fprintf([reverseStr, msg]);
    fprintf('%%');
    reverseStr = repmat(sprintf('\b'), 1, length(msg)+1);
    
end
clear eps
runtime = toc(tMC);
fprintf([reverseStr, 'Done\n\n']);
fprintf(['\tMonte Carlo completed in ',num2str(runtime),' seconds\n\n']);


% reset probabilities: P(tau < Mdt| Z= 0)
if(sim.M > 1)
    for m = 1:sim.M
        count = 0;
        for b = 1:B
            z_prev = zeros(sim.nx,1);

            %-- Stopping Times --%
            if(strcmp(sim.MC_type,'STO'))
                % run error process M steps and if its norm exceeds delta 
                % at any point increment count
                for k = 1:m
                    % update stochastic error process
%                     eps =  sqrt(sim.Q)*randn(sim.nx,1);
%                     z = sim.A*z_prev + eps;
                    z = (eye(size(Ac,1)) + Ac*sim.dt)*z_prev + Qc*W(:,k,b); 
                    z_prev = z;
                    if(norm(z) >= sim.delta)
                        count = count + 1;
                        break;
                    end
                end

            %-- Error --%
            elseif(strcmp(sim.MC_type,'ERR'))
                % only checks error after M steps
                for k = 1:m
                    % update stochastic error process
                    eps =  sqrt(sim.Q)*randn(sim.nx,1);
                    z = sim.A*z_prev + eps; 
                    z_prev = z;
                end
                if(norm(z) >= sim.delta)
                    count = count + 1;
                end
            end
        end
        reset_prob(m) = count/B;    
    end
end

% Store and save lookup table
fprintf('\tSaving Monte Carlo lookup table...');
cd(folder);
save(file1,'prob');
save(file2,'reset_prob');
cd('../');
fprintf('Done\n');