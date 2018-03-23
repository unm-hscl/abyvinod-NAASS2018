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
pareto_optim_indx = [1,6,9,12];
fontSizeVal = 25;
load('data\20171218_pareto_0x05.mat')
mat_control_effort_optimal_norm(1,:) = control_effort_optimal_norm;
mat_safety_probability_optimal(1,:) = safety_probability_optimal;
load('data\20171218_pareto_0x0625.mat')
mat_control_effort_optimal_norm(2,:) = control_effort_optimal_norm;
mat_safety_probability_optimal(2,:) = safety_probability_optimal;
load('data\20171218_pareto_0x075.mat')
mat_control_effort_optimal_norm(3,:) = control_effort_optimal_norm;
mat_safety_probability_optimal(3,:) = safety_probability_optimal;
load('data\20171218_pareto_0x1.mat')
mat_control_effort_optimal_norm(4,:) = control_effort_optimal_norm;
mat_safety_probability_optimal(4,:) = safety_probability_optimal;
load('data\20171218_pareto_0x5.mat')
mat_control_effort_optimal_norm(5,:) = control_effort_optimal_norm;
mat_safety_probability_optimal(5,:) = safety_probability_optimal;
upper_limit_y=-log10(min(min(safety_probability_optimal)))+1;
lower_limit_y=-log10(max(max(safety_probability_optimal))*1.01);
axis_vec = [0.1 norm(umax*ones(1,2*last_time_step)) lower_limit_y upper_limit_y];

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
