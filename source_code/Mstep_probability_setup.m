function sim = Mstep_probability_setup(varargin)
% Function to either load or compute the lookup tables for the desired
% prediction horizon
% 
% Inputs:   
%   sim           structure containing the simulation parameters
%   sched_delay   scheduling delay used in ST design

% Outputs:      
%   sim           update simulation parameter structure with the lookup
%                 tables
%
%--------------------------------------------------------------------------

sim = varargin{1};
    
s = 'stable';
lambda = abs(eig(sim.A));
for i = 1:length(lambda)
    if(lambda(i) >= 1)
        s = 'unstable';
        break;
    end
end
if(strcmp(sim.SIM_TYPE,'STOCHASTIC'))
    if(sum(diag(sim.A)) ~= 0.95*sim.nx && strcmp(s,'stable'))
        s = [s, '_rand'];
    end
    if(sum(diag(sim.A)) ~= 1.1*sim.nx && strcmp(s,'unstable'))
        s = [s, '_rand'];
    end
end

if(strcmp(sim.FET_TYPE,'MC'))
%%-------------------------------------------------------------------------
% Monte Carlo Simulation Solution
%--------------------------------------------------------------------------
sim.numSim = 1e4;            % number of MC simulation
sim.MC_type = 'STO';      % Monte Carlo Type {'STO','ERR'}


if(strcmp(sim.TRIG_TYPE,'PT') || strcmp(sim.TRIG_TYPE,'PPT'))
    temp = sim.M;
    for m = 1:sim.M    
        if(strcmp(sim.SIM_TYPE,'STOCHASTIC'))
            folder = 'Stochastic Processes';

            % check if lookup table already exists and if not run a Monte Carlo
            % simulation to generate it
            filename1 = ['Mstep_lookup_',sim.FET_TYPE,'_',sim.MC_type,'_',num2str(m),'M_',...
                        num2str(sim.nx),'D_',num2str(length(sim.Z0_set)),'z0_',s,'.mat'];
            filename2 = ['reset_lookup_',sim.MC_type,'_',...
                            num2str(m),'M_',num2str(sim.nx),'D_',s,'.mat'];

        elseif(strcmp(sim.SIM_TYPE,'AV'))
            folder = 'Autonomous Vehicles';
            filename1 = ['Mstep_lookup_AV_',sim.FET_TYPE,'_',sim.MC_type,'_',num2str(m),'M_',...
                        num2str(sim.nx),'D_',num2str(length(sim.Z0_set)),'z0_',s,'.mat'];
            filename2 = ['reset_lookup_AV_',sim.MC_type,'_',...
                            num2str(m),'M_',num2str(sim.nx),'D','.mat'];
        
        elseif(strcmp(sim.SIM_TYPE,'CP'))
            folder = 'Cart-Pole';
            filename1 = ['Mstep_lookup_CP_',sim.FET_TYPE,'_',sim.MC_type,'_',num2str(m),'M_',...
                        num2str(sim.nx),'D_',num2str(length(sim.Z0_set)),'z0_',s,'.mat'];
            filename2 = ['reset_lookup_CP_',sim.MC_type,'_',...
                            num2str(m),'M_',num2str(sim.nx),'D','.mat'];

        end
        
        f1 = exist(filename1,'file');
        f2 = exist(filename2,'file');
        if(f1 == 0 || f2 == 0)
            sim.M = m;
            run_Monte_Carlo(sim,filename1,filename2,folder);
        end

        P = load(filename1);
        prob{m} = P.prob;
        if(m == sim.M)
            load(filename2);
        end
        
%         if nargin < 2
%             sim.lookup_prob = prob;
%             sim.reset_prob = reset_prob;
%         else
%             mc = varargin{2};
%             if(strcmp(mc,'STO'))
%                 sim.lookup_prob_sto = prob;
%             elseif(strcmp(mc,'ERR'))
%                 sim.lookup_prob_err = prob;
%             end 
%         end         
    end
    sim.M = temp;
    sim.lookup_prob = prob;
    sim.reset_prob = reset_prob;
    clear prob reset_prob P; 
    
elseif(strcmp(sim.TRIG_TYPE,'ST')||strcmp(sim.TRIG_TYPE,'ST2'))
    
    % lookup table to determine M
    if(strcmp(sim.SIM_TYPE,'STOCHASTIC'))
        folder = 'Stochastic Processes';
        filename1 = ['ST_NUM_lookup_1D.mat'];
%         filename1 = [sim.TRIG_TYPE,'_',sim.FET_TYPE,'_lookup_',num2str(sim.nx),'D.mat'];
    elseif(strcmp(sim.SIM_TYPE,'AV'))
        folder = 'Autonomous Vehicles';
        filename1 = [sim.TRIG_TYPE,'_AV_lookup',num2str(sim.nx),'D.mat'];
    end

       
    f = exist(filename1,'file');
    if(f == 0)
        error('No file for ST M lookup');
%         getExitProbPDE(sim,filename1,folder);
    end
    
    load(filename1);
    sim.ST_M_lookup = prob;
    clear prob;
    
    % lookup table for scheduling delay
    temp = sim.M;
    sim.M = varargin{2};
    if(strcmp(sim.SIM_TYPE,'STOCHASTIC'))
        filename2 = ['Mstep_lookup_',sim.FET_TYPE,'_',sim.MC_type,'_',num2str(sim.M),...
                        'M_',num2str(sim.nx),'D_',num2str(length(sim.Z0_set)),'z0_',s,'.mat'];
    elseif(strcmp(sim.SIM_TYPE,'AV'))
        filename2 = ['Mstep_lookup_AV_',sim.FET_TYPE,'_',num2str(sim.M),'M_',...
                       num2str(sim.nx),'D_',num2str(length(sim.Z0_set)),'z0_',s,'.mat'];
    end
    
    f = exist(filename2,'file');
    if(f == 0)
        sim.TRIG_TYPE = 'PT';
        getExitProbPDE(sim,filename2,folder);
        sim.TRIG_TYPE = 'ST2';
    end
    sim.M = temp;
    
    load(filename2);
    sim.sched_delay_lookup = prob;
    clear prob;
    
    
    % lookup table for when agent changes to PT
    prob = [];
    for m = 1:sim.M    
        if(strcmp(sim.SIM_TYPE,'STOCHASTIC'))
            filename3 = ['Mstep_lookup_',sim.FET_TYPE,'_',sim.MC_type,'_',num2str(m),...
                            'M_',num2str(sim.nx),'D_',num2str(length(sim.Z0_set)),'z0_',s,'.mat'];
        elseif(strcmp(sim.SIM_TYPE,'AV'))
            filename3 = ['Mstep_lookup_AV_',sim.FET_TYPE,'_',num2str(m),'M_',...
                           num2str(sim.nx),'D_',num2str(length(sim.Z0_set)),'z0_',s,'.mat'];
        end

        f = exist(filename3,'file');
        if(f == 0)
            sim.TRIG_TYPE = 'PT';
            getExitProbPDE(sim,filename3,folder);
            sim.TRIG_TYPE = 'ST2';
        end
        
        % lookup tables for m-step probs
        P = load(filename3);
        prob{m} = P.prob;        
    end
    
    sim.lookup_prob = prob;
    clear prob P;    
end


elseif(strcmp(sim.FET_TYPE,'NUM'))
%%-------------------------------------------------------------------------
% Numerical Solution
%--------------------------------------------------------------------------
if(strcmp(sim.TRIG_TYPE,'PT') || strcmp(sim.TRIG_TYPE,'PPT'))
    sim.MC_type = 'STO';      % Monte Carlo Type {'STO','ERR'}
    temp = sim.M;
    for m = 1:sim.M 
        if(strcmp(sim.SIM_TYPE,'STOCHASTIC'))
            folder = 'Stochastic Processes';
            filename1 = ['Mstep_lookup_',sim.FET_TYPE,'_',num2str(sim.M),...
                            'M_',num2str(sim.nx),'D_',num2str(length(sim.Z0_set)),'z0_',s,'.mat'];
            filename2 = ['reset_lookup_',sim.MC_type,'_',...
                            num2str(sim.M),'M_',num2str(sim.nx),'D','.mat'];

        elseif(strcmp(sim.SIM_TYPE,'AV'))
            folder = 'Autonomous Vehicles';
            filename1 = ['Mstep_lookup_AV_',sim.FET_TYPE,'_',num2str(m),'M_',...
                        num2str(sim.nx),'D_',num2str(length(sim.Z0_set)),'z0_',s,'.mat'];
            filename2 = ['reset_lookup_AV_',sim.MC_type,'_',...
                            num2str(m),'M_',num2str(sim.nx),'D','.mat'];
            
%             filename1 = ['Mstep_lookup_AV_',sim.FET_TYPE,'_',num2str(sim.M),'M_',...
%                         num2str(sim.nx),'D_',num2str(length(sim.Z0_set)),'z0_',s,'.mat'];
%             filename2 = ['reset_lookup_AV_',sim.MC_type,'_',...
%                             num2str(sim.M),'M_',num2str(sim.nx),'D','.mat'];
        end

        f1 = exist(filename1,'file');
        f2 = exist(filename2,'file');
        if(f1 == 0 || f2 == 0)
            sim.M = m;
            getExitProbPDE(sim,filename1,filename2,folder);
        end

        P = load(filename1);
        prob{m} = P.prob;
        R = load(filename2);
        reset(m) = R.reset_prob;

    end
    sim.M = temp;
    sim.lookup_prob = prob;
    sim.reset_prob = reset;
    clear prob reset_prob; 
    
elseif(strcmp(sim.TRIG_TYPE,'ST')||strcmp(sim.TRIG_TYPE,'ST2'))
    % lookup table to determine M
    if(strcmp(sim.SIM_TYPE,'STOCHASTIC'))
        folder = 'Stochastic Processes';
        filename1 = [sim.TRIG_TYPE,'_',sim.FET_TYPE,'_lookup_',num2str(sim.nx),'D.mat'];
    elseif(strcmp(sim.SIM_TYPE,'AV'))
        folder = 'Autonomous Vehicles';
        filename1 = [sim.TRIG_TYPE,'_',sim.FET_TYPE,'_AV_lookup',num2str(sim.nx),'D.mat'];
    end
    
    f = exist(filename1,'file');
    if(f == 0)
        getExitProbPDE(sim,filename1,folder);
    end
    
    load(filename1);
    sim.ST_M_lookup = prob;
    clear prob;
    
    
    % lookup table for scheduling delay    
    temp = sim.M;
    sim.M = varargin{2};
    if(strcmp(sim.SIM_TYPE,'STOCHASTIC'))
        filename2 = ['Mstep_lookup_',sim.FET_TYPE,'_',num2str(sim.M),...
                        'M_',num2str(sim.nx),'D_',num2str(length(sim.Z0_set)),'z0_',s,'.mat'];
    elseif(strcmp(sim.SIM_TYPE,'AV'))
        filename2 = ['Mstep_lookup_AV_',sim.FET_TYPE,'_',num2str(sim.M),'M_',...
                       num2str(sim.nx),'D_',num2str(length(sim.Z0_set)),'z0_',s,'.mat'];
    end
    
    f = exist(filename2,'file');
    if(f == 0)
        sim.TRIG_TYPE = 'PT';
        getExitProbPDE(sim,filename2,folder);
        sim.TRIG_TYPE = 'ST2';
    end
    sim.M = temp;
    
    load(filename2);
    sim.sched_delay_lookup = prob;
    clear prob;
    
    
    % lookup table for when agent changes to PT
    for m = 1:M
        if(strcmp(sim.SIM_TYPE,'STOCHASTIC'))
            filename3 = ['Mstep_lookup_',sim.FET_TYPE,'_',num2str(m),...
                            'M_',num2str(sim.nx),'D_',num2str(length(sim.Z0_set)),'z0_',s,'.mat'];
        elseif(strcmp(sim.SIM_TYPE,'AV'))
            filename3 = ['Mstep_lookup_AV_',sim.FET_TYPE,'_',num2str(m),'M_',...
                           num2str(sim.nx),'D_',num2str(length(sim.Z0_set)),'z0_',s,'.mat'];
        end

        f = exist(filename3,'file');
        if(f == 0)
            sim.TRIG_TYPE = 'PT';
            getExitProbPDE(sim,filename3,folder);
            sim.TRIG_TYPE = 'ST2';
        end
        
        % lookup table for m-step prob
        P = load(filename3);
        prob{m} = P.prob;
    end
    sim.lookup_prob = prob;
    clear prob P;
end
end
