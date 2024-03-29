%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Name of the programmer: Abraham %
% Date: 2018-03-23                %
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%% Purpose
% Generate Figure 5 in Subsection 4.3

%% Notes
% Requires a data file in data\ folder and two gamma indices that would be treated as the knee and the safe knee
% Run scan_through_umax.m to get the data files

clear
clc

%% Edit this variable
date_of_interest = '20171218'; %20180324'

%% Data and plotting params
pareto_optim_indx = [1,6,9,12];
fontSizeVal = 25;
umax_values = [0.05, 0.0625, 0.075, 0.1, 0.5];
datenowstr_values = {'0x05.mat','0x0625.mat','0x075.mat','0x1.mat','0x5.mat'};
elapsed_time_tot = 0;

%% Iterate over all data files to get data
for umax_indx = 1:length(umax_values)
    % Load data
    date_of_interest_str = strcat('data/',date_of_interest,'_pareto_',datenowstr_values{umax_indx}); 
    load(date_of_interest_str);
    % Get required data
    mat_control_effort_optimal_norm(umax_indx,:) = control_effort_optimal_norm;
    mat_safety_probability_optimal(umax_indx,:) = safety_probability_optimal;
    elapsed_time_tot = elapsed_time_tot + sum(elapsed_time);
    umax = umax_values(umax_indx);
end

figure(105)
clf
plot(mat_control_effort_optimal_norm(1,1:end-1),mat_safety_probability_optimal(1,1:end-1),'rs--','MarkerSize',10)
hold on                                                                                    
plot(mat_control_effort_optimal_norm(2,1:end-1),mat_safety_probability_optimal(2,1:end-1),'bo--','MarkerSize',10)
plot(mat_control_effort_optimal_norm(3,1:end-1),mat_safety_probability_optimal(3,1:end-1),'md--','MarkerSize',10)
plot(mat_control_effort_optimal_norm(4,1:end-1),mat_safety_probability_optimal(4,1:end-1),'g^--','MarkerSize',10)
plot(mat_control_effort_optimal_norm(5,1:end-1),mat_safety_probability_optimal(5,1:end-1),'k*--','MarkerSize',10)
leg=legend('$\bar{u}_\mathrm{bound}=0.05$','$\bar{u}_\mathrm{bound}=0.0625$','$\bar{u}_\mathrm{bound}=0.075$','$\bar{u}_\mathrm{bound}=0.1$','$\bar{u}_\mathrm{bound}=0.5$');
set(leg,'interpreter','latex','Location','SouthEast');
set(gca,'FontSize',fontSizeVal);
ylabel('Reach-avoid probability','interpreter','latex')
xlabel('Control effort','interpreter','latex')  %(${\Vert U \Vert}_2$)
box on
grid on
savefig(gcf,'figs\VaryingU_pareto.fig','compact')
saveas(gcf,'figs\VaryingU_pareto.png')
