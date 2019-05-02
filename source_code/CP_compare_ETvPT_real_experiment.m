%% Comparison script between ET and PT real experiments
clear;
clc;

cutTime = 5;            % [sec]
N = 10;                 % # of agents
dt = 0.01;              % time step size
T = 30;                 % simulation time
t = 0:dt:T;             % time vector
nx = 4;
SYNC = 1;
P_CONSTRAINT = 0;
CAST_FLAG = 1;
DIST_FLAG = 0;          % {0 = disturbance off, 1 = disturbances on}
REF_FLAG = 0;           % {0 = reference off, 1 = reference on}
% if(CAST_FLAG && P_CONSTRAINT)
%     trig_type = {'PText','ET1','ET2'};   
% else
%     trig_type = {'PT','ET1','ET2'};
% end
    trig_type = {'PT','PText','ET1','ET2'};
Kset = [3,4,6,8,10];
c = 0.5;
cs = num2str(c);
dot_id = find(cs=='.');
cs = cs(dot_id+1:end);

thisPath = pwd;
if(SYNC)
    addpath([thisPath, '/sync_data_430']);
else
    if(DIST_FLAG == 0 && REF_FLAG == 0)
        addpath([thisPath, '/data_refOFF_distOFF']);    
    elseif(DIST_FLAG == 1 && REF_FLAG == 0)
         addpath([thisPath, '/data_refOFF_distON']);       
    elseif(DIST_FLAG == 0 && REF_FLAG == 1)
        addpath([thisPath, '/data_refON_distOFF']);    
    else
        error('No data!');
    end
end

Zbar = zeros(length(trig_type),length(Kset));
Zvar = zeros(length(trig_type),length(Kset));
Ebar = zeros(length(trig_type),length(Kset));
Evar = zeros(length(trig_type),length(Kset));
mean_util = zeros(length(trig_type),length(Kset));

m = zeros(1,3);
for tr = 1:length(trig_type)
    for k = 1:length(Kset)
        for n = 1:3         % number of trials
            if(tr == 1 || tr == 2)
                filename_agents = ['AGENT_DATA_',trig_type{tr},'_K',num2str(Kset(k)),'_c0',cs,'_',num2str(n),'.mat'];
                load(filename_agents);
                if(n == 1)
                    filename_net = ['NET_DATA_',trig_type{tr},'_K',num2str(Kset(k)),'_c0',cs,'.mat'];
                    load(filename_net);
                end
            else
                filename_agents = ['AGENT_DATA_',trig_type{tr},'_K',num2str(Kset(k)),'_',num2str(n),'.mat'];
                load(filename_agents);            
            end

            d = (size(data,1)-1)/N;         % # of data vectors
            AGENTS = cell(N,1);
            for ID = 1:N
                if(SYNC)
                    temp = find(data(38,:)>0);
%                     start_idx = temp(1);
                    start_idx = 3001;       % take last 30s
                    tvec = t(start_idx:end);
                else
                    start_idx = 1;
                    if(size(AGENTS{ID}.x(1,:),2) == length(t))
                        tvec = t;
                    else
                        tvec = t(1:end-1);
                    end
                end           
                idx = d*(ID-1)+2:d*ID+1;
                agent_data = data(idx,:);
                AGENTS{ID}.x = agent_data(1:4,start_idx:end);
                AGENTS{ID}.xhat = agent_data(5:8,start_idx:end);
                AGENTS{ID}.xref = agent_data(9,start_idx:end);
                AGENTS{ID}.u_stable = agent_data(10,start_idx:end);
                AGENTS{ID}.u_noisy = agent_data(11,start_idx:end);    
                AGENTS{ID}.z = agent_data(12:15,start_idx:end);
                AGENTS{ID}.P = agent_data(16,start_idx:end);
                AGENTS{ID}.comm_slot = agent_data(17,start_idx:end);
                AGENTS{ID}.comm_trig = agent_data(18,start_idx:end);
                AGENTS{ID}.comm_trig_util = agent_data(18,:);

                % cap state when pendulum goes unstable
    %             if(ID == 1 && k == 1)
    %                 ids = find(AGENTS{ID}.x(2,:) >= 3);
    %                 if(~isempty(ids))
    %                     AGENTS{ID}.x(:,ids(1)+1:length(tvec)+1) = [AGENTS{ID}.x(1:3,ids(1)).*ones(3,length(tvec)+1-ids(1)); THETA_MAX*ones(1,length(tvec)+1-ids(1))];
    %                 end
    %             end
            end
            
            if(tr == 1 || tr == 2)
                if(n == 1)
                    Np = net_data(2,start_idx:end);
                end
            elseif(tr == 4)
                Np = N;
            else
                Np = zeros(1,length(t));
            end
            
            % Bandwidth
            if(tr == 2)
                P_CONSTRAINT = 1;
            else
                P_CONSTRAINT = 0;
            end
            if(P_CONSTRAINT)
                b = (Kset(k)*4*nx + (4+2)*N)/dt;
            else
                b = (Kset(k)*4*nx + 4*N)/dt;
            end
%             b = (Kset(k)*4*nx + 4*N)/dt;
            % capacity per time step [bytes]
            capacity = dt*b;
            u(n) = mean(util(AGENTS,Np,start_idx,nx,capacity,tr,CAST_FLAG));
            
            [zbar,zvar,ebar,evar] = men(AGENTS,SYNC);
            m(n) = ebar;
        end
        
%         [zbar,zvar,ebar,evar] = men(AGENTS,SYNC);
%         Zbar(tr,k) = zbar; Zvar(tr,k) = zvar;
%         Ebar(tr,k) = ebar; Evar(tr,k) = evar; 
        Ebar(tr,k) = mean(m);
        mean_util(tr,k) = mean(u);
        clear AGENTS;
        fclose('all');
    end
end

% Plot
pos = [0 0 813 645];
plot_settings(18,1.5);
% estimation error
% figure('Position',pos);
% hold on;grid on; box on;
% for tr = 1:length(trig_type)
%     plot(Kset, Zbar(tr,:),'-s');
% %     errorbar(K,Zbar(tr,:),Zvar(tr,:),'-s');
% end
% xlabel('Number of communication slots [K]');
% ylabel('Mean Estimation Error Norm [$\bar{Z}$]');
% title(['Experiment Trigger Comparison, N = ',num2str(N)]);
% l = legend(trig_type);
% l.Orientation = 'horizontal';
% l.Location = 'northeast';
% xlim([0 Kset(end)+2]);

% control error
fig = figure('Position',pos);
color = [0 0 0];
hold on;grid on; box on;
if(SYNC)
    xlim([0 Kset(end)+1]);
    xlabel('Number of communication slots $K$');
    title(['Experiment Trigger Comparison, N = ',num2str(N),', c = ',num2str(c)]);
    for tr = 1:length(trig_type)
        plot(Kset,Ebar(tr,:),'-s');
    end
    l = legend(trig_type);

else
    for tr = 1:length(trig_type)
    %     plot(Kset, Ebar(tr,:),'-s');
        errorbar(Kset,Ebar(tr,:),Evar(tr,:),'-s');
    end
    xlabel('Number of communication slots $K$');
    ylabel('Mean Control Error Norm $\bar{E}$');
    title(['Experiment Trigger Comparison, N = ',num2str(N)]);
    l = legend(trig_type);
    l.Orientation = 'horizontal';
    l.Location = 'northeast';
    xlim([2 Kset(end)+1]);
end

% network utilization
figure('Position',pos);
hold on;grid on; box on;
for tr = 1:length(trig_type)
    plot(Kset,mean_util(tr,:),'-s');
end
xlabel('Number of communication slots $K$');
ylabel('Mean Network Utilization [$\%$]');
title(['Trigger comparison, N = ',num2str(N)]);
l = legend(trig_type);
xlim([2 Kset(end)+1]);


if(SYNC)
    rmpath([thisPath, '/sync_data']);
else
    if(DIST_FLAG == 0 && REF_FLAG == 0)
        rmpath([thisPath, '/data_refOFF_distOFF']);    
    elseif(DIST_FLAG == 1 && REF_FLAG == 0)
         rmpath([thisPath, '/data_refOFF_distON']);       
    elseif(DIST_FLAG == 0 && REF_FLAG == 1)
        rmpath([thisPath, '/data_refON_distOFF']);
    else
        error('No data!');
    end
end


% computes the mean estimation and control error norms (+ variances)
function [mean_est_error_norm,var_est_error_norm,mean_ctrl_error_norm,var_ctrl_error_norm] = men(AGENTS,sync)
    if(sync)
        sim_znorms = []; sim_enorms = [];
        ref = AGENTS{3}.x(1,:);
        for i = 1:length(AGENTS)
            if(i ~= 3)
                sim_enorms = [sim_enorms; norm(AGENTS{i}.x(1,:) - ref)];
            end
            est_norm = sqrt(sum(AGENTS{i}.z.*AGENTS{i}.z));
            sim_znorms = [sim_znorms; est_norm];     
        end
        mean_z_agents = mean(sim_znorms,2);
        mean_est_error_norm = mean(mean_z_agents);
        var_est_error_norm = mean(std(sim_znorms,0,2));
        mean_ctrl_error_norm = mean(sim_enorms);
        var_ctrl_error_norm = mean(std(sim_enorms,0,2));
        
    else
       sim_znorms = []; sim_enorms = [];
        for i = 1:length(AGENTS)
            if(i ~= 1)
                ctrl_norm = sqrt(sum(AGENTS{i}.x.*AGENTS{i}.x));
                sim_enorms = [sim_enorms; ctrl_norm];
                est_norm = sqrt(sum(AGENTS{i}.z.*AGENTS{i}.z));
                sim_znorms = [sim_znorms; est_norm];
            end
        end
        mean_z_agents = [mean(sqrt(sum(AGENTS{1}.z.*AGENTS{1}.z))); mean(sim_znorms,2)];
        mean_est_error_norm = mean(mean_z_agents);
        var_est_error_norm = mean([std(mean(sqrt(sum(AGENTS{1}.z.*AGENTS{1}.z)))); std(sim_znorms,0,2)]);
        mean_e_agents = [mean(sqrt(sum(AGENTS{1}.x.*AGENTS{1}.x))); mean(sim_enorms,2)];
        mean_ctrl_error_norm = mean(mean_e_agents);
        var_ctrl_error_norm = mean(std(sim_enorms,0,2));
    end
end

% calculate network utilization
function U = util(AGENTS,Np,idx,nx,capacity,tr,cast)
    commDecisions = [];
    for i = 1:length(AGENTS)
        commDecisions = [commDecisions; AGENTS{i}.comm_trig_util(idx:end)];
    end
    Nc = sum(commDecisions,1);
    if(tr == 1 || tr == 2) %PT, PText, ET2
        if(cast)
            U = 100*(Np + 4*nx*Nc)/capacity;
        else
            U = 100*(4*Np + 4*nx*Nc)/capacity;
        end
    else
        U = 100*(4*Np + 4*nx*Nc)/capacity;
    end
    
%     if(P_CONSTRAINT||TRIG_TYPE == 4)
%         net_util = 100*((4+2)*Np + 4*nx*Nc)/capacity;         
%     else     
%     end
end