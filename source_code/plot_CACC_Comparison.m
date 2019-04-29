% Script to plot CACC comparison between ET1, ET2, ET3 and PT (w/ and w/o
% extension)
% 
% ET1 = slots assigned randomly at each timestep
% ET2 = slots assigned to K random agents for all time steps
% ET3 = slots assigned to vehicles at the beginning of the platoons
% PT = slots assigned M-steps in advance based on predictions
%
%--------------------------------------------------------------------------

clear;
clc;

N1 = [20,30,40,50,100];
N2 = [20,30,40,50];
K = 20;
trig_type1 = {'ET1','ET2','ET3','PT','PT*'};
trig_type2 = {'ET1','PT','PT*'};
mean_error_norm = nan(length(trig_type1),length(N1));
mean_util = nan(length(trig_type1),length(N1));
% load data
% for n = 1:length(N2)
for n = 1:length(N1)       
    file{1} = ['CACC_ET1pred_var2_K',num2str(K),'_N',num2str(N1(n)),'.mat'];
    file{4} = ['CACC_PT_K',num2str(K),'_N',num2str(N1(n)),'.mat'];
    file{5} = ['CACC_PTextended_S1A3_var2_K',num2str(K),'_N',num2str(N1(n)),'.mat'];

    if(n ~= length(N1))
        file{2} = ['CACC_ET2pred_var2_K',num2str(K),'_N',num2str(N1(n)),'.mat'];
%         file{3} = ['CACC_ET3_K',num2str(K),'_N',num2str(N1(n)),'.mat'];        
%         triggers = [1,2,3,4,5];
        triggers = [1,2,4,5];
    else
        triggers = [1,4,5];
    end
    
    for tr = triggers
        load(file{tr});

        % mean error norms
        for ID = 1:sim.N
            ebar(tr,ID) = AGENTS(ID).mean_ctrlError_norm(sim);
        end
        mean_error_norm(tr,n) = mean(ebar(tr,:));
        stdev_error_norm(tr,n) = std(ebar(tr,:));

        % get utilization
        mean_util(tr,n) = mean(net.util);

        clear AGENTS net sim   
    end 
    clear file
    fclose('all');
end


%% Plot
pos = [759 302 813 645];
plot_settings(18,1.5);

fig = figure('Position',pos);
color = [0 0 0];
set(fig,'defaultAxesColorOrder',[color; color]);
hold on;hold on;grid on; box on;
xlim([15 N1(end)+5]);
xlabel('Number of Agents [N]');
yyaxis left;
title(['Trigger Comparison, K = ',num2str(K)]);
% plot mean
% plot(N1,mean_error_norm(4,:),'-s','MarkerSize',8,'LineWidth',1.5,'Color',[0 0.4470 0.7410]);
plot(N1,mean_error_norm(5,:),'-s','MarkerSize',8,'LineWidth',1.5,'Color',[0 0.4470 0.7410]);
plot(N1,mean_error_norm(1,:),'-s','MarkerSize',8,'LineWidth',1.5,'Color',[0.85 0.3250 0.0980]);
ylabel('Mean Control Error Norm $\bar{E}$');
yyaxis right;
plot(N1,mean_error_norm(2,:),'-*','MarkerSize',8,'LineWidth',1.5,'Color',[0.9290 0.6490 0.1250]);
plot(N1,mean_error_norm(3,:),'-*','MarkerSize',8,'LineWidth',1.5,'Color',[0.4940 0.1840 0.5560]);
% plot variance
% plot(N1,stdev_error_norm(4,:),'-s','MarkerSize',8,'LineWidth',1.5,'Color',[0 0.4470 0.7410]);
% plot(N1,stdev_error_norm(5,:),'-s','MarkerSize',8,'LineWidth',1.5,'Color',[0 0.4470 0.7410]);
% plot(N1,stdev_error_norm(1,:),'-s','MarkerSize',8,'LineWidth',1.5,'Color',[0.85 0.3250 0.0980]);
% ylabel('Mean Control Error Norm $\bar{E}$');
% yyaxis right;
% plot(N1,stdev_error_norm(2,:),'-*','MarkerSize',8,'LineWidth',1.5,'Color',[0.9290 0.6490 0.1250]);
% plot(N1,stdev_error_norm(3,:),'-*','MarkerSize',8,'LineWidth',1.5,'Color',[0.4940 0.1840 0.5560]);
% plot errorbars
% errorbar(N1,mean_error_norm(5,:),stdev_error_norm(4,:),'-s','Color',[0.85 0.3250 0.0980]);
% errorbar(N1,mean_error_norm(1,:),stdev_error_norm(1,:),'-s','Color',[0 0.4470 0.7410]);
% ylabel('Mean Control Error Norm $\bar{E}$');
% yyaxis right;
% plot(N1,mean_error_norm(2,:),'-*','MarkerSize',8,'LineWidth',1.5,'Color',[0.9290 0.6490 0.1250]);
% errorbar(N,mean_error_norm(2,:),stdev_error_norm(2,:),'-*','Color',[0.9290 0.6490 0.1250]);
% errorbar(N,mean_error_norm(3,:),stdev_error_norm(3,:),'-*','Color',[0.4940 0.1840 0.5560]);
temp = {'PT','ET1','ET2','ET3'};
l = legend(temp);
l.Orientation = 'horizontal';
l.Location = 'northwest';

figure('Position',pos);
hold on;grid on; box on;
% plot(N1,mean_util(4,:),'-s','LineWidth',1.5);
plot(N1,mean_util(5,:),'-s','LineWidth',1.5);
plot(N1,mean_util(1,:),'-s','LineWidth',1.5);
plot(N1,mean_util(2,:),'-s','LineWidth',1.5);
plot(N1,mean_util(3,:),'-s','LineWidth',1.5);
% for tr = 1:length(trig_type)
%     plot(N,temp(tr,:),'-s','LineWidth',1.5);
% end
xlabel('Number of Agents [N]');
ylabel('Mean Network Utilization $\bar{U}$ [$\%$]');
title(['Trigger comparison, K = ',num2str(K)]);
l = legend(temp);
l.Orientation = 'horizontal';
l.Location = 'northwest';
xlim([15 N1(end)+5]);