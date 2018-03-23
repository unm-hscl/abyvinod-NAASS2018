%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Name of the programmer: Abraham %
% Date: 2018-03-23                %
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%% Purpose
% Generate Figures 1 to 4 in Subsection 4.2

%% Notes
% Requires a data file in data\ folder and two gamma indices that would be
% treated as the knee and the safe knee

clear
clc
close all

load('data\20171218_pareto_0x1.mat')
pareto_optim_indx_Knee = 15;
pareto_optim_indx_Knee_safe = 10;

%load('data\20182623_110314pareto_umax_as_0x1.mat');
%pareto_optim_indx_Knee = 15;
%pareto_optim_indx_Knee_safe = 9;

pareto_optim_indx_onlySafety = 1;
fontSizeVal = 25;
color_onlySafety = 'r';
color_Knee = 'g';
color_Knee_safe = 'b';
upper_limit_y=-log10(min(safety_probability_optimal(1:end-1)))+1;
lower_limit_y=-log10(max(safety_probability_optimal)*1.01);
axis_vec = [0.1 norm(umax*ones(1,last_time_step)) lower_limit_y upper_limit_y];

%% Plot of various solutions
figure(101)
clf
box on;
hold on;
plot(polytope_safe_set.slice([3,4], slice_at_vx_vy), 'color', 'w');
plot(polytope_target_set.slice([3,4], slice_at_vx_vy), 'color', 'y');
set(gca,'FontSize',25)
trajectory_onlySafety = reshape(concatenated_A_matrix(5:end,:)*initial_state + H_matrix_no_initial_state*control_effort_optimal(:,pareto_optim_indx_onlySafety),4,[]); 
trajectory_Knee = reshape(concatenated_A_matrix(5:end,:)*initial_state + H_matrix_no_initial_state*control_effort_optimal(:,pareto_optim_indx_Knee),4,[]); 
trajectory_Knee_safe = reshape(concatenated_A_matrix(5:end,:)*initial_state + H_matrix_no_initial_state*control_effort_optimal(:,pareto_optim_indx_Knee_safe),4,[]); 
trajectory_mean_safe = reshape(concatenated_A_matrix(5:end,:)*initial_state + H_matrix_no_initial_state*initial_guess_for_input_vector,4,[]); 
trajectory_input = reshape(concatenated_A_matrix(5:end,:)*initial_state,4,[]); 
plot(trajectory_onlySafety(1,:),trajectory_onlySafety(2,:),strcat(color_onlySafety,'s--'),'MarkerSize',15);
plot(trajectory_Knee_safe(1,:),trajectory_Knee_safe(2,:),strcat(color_Knee_safe,'d--'),'MarkerSize',15);
plot(trajectory_Knee(1,:),trajectory_Knee(2,:),strcat(color_Knee,'o--'),'MarkerSize',15);
plot(trajectory_mean_safe(1,:),trajectory_mean_safe(2,:),'m*--','MarkerSize',15);
plot(trajectory_input(1,:),trajectory_input(2,:),'c^--','MarkerSize',15);
%scatter(trajectory_onlySafety(1,:),trajectory_onlySafety(2,:),200,'filled','s','MarkerFaceColor',color_onlySafety);
%scatter(trajectory_Knee_safe(1,:),trajectory_Knee_safe(2,:),200,'filled','d','MarkerFaceColor',color_Knee_safe);
%scatter(trajectory_Knee(1,:),trajectory_Knee(2,:),200,'filled','s','MarkerFaceColor',color_Knee);
%scatter(trajectory_input(1,:),trajectory_input(2,:),200,'filled','t','MarkerFaceColor','k');
axis([-1.1 1.1 -1 0])
leg=legend('Safe','Target','No control effort optimization', 'Higher safety compromise', 'Lesser control effort compromise','Initial trajectory for optimization','No safety optimization (${\Vert \rho^\ast \Vert}_2=0$)');
set(leg,'interpreter','latex','Location','West');
savefig(gcf,'figs\LOS_trajectory.fig','compact')
saveas(gcf,'figs\LOS_trajectory.png')

%% Pareto optimal curve normal
figure(102)
clf
plot(control_effort_optimal_norm,safety_probability_optimal,'k*--','MarkerSize',15)
plot(control_effort_optimal_norm(1:end-1),safety_probability_optimal(1:end-1),'k*--','MarkerSize',15)
hold on
scatter(control_effort_optimal_norm(pareto_optim_indx_onlySafety),safety_probability_optimal(pareto_optim_indx_onlySafety),200,'filled','s','MarkerFaceColor',color_onlySafety)
scatter(control_effort_optimal_norm(pareto_optim_indx_Knee_safe),safety_probability_optimal(pareto_optim_indx_Knee_safe),200,'filled','d','MarkerFaceColor',color_Knee_safe)
scatter(control_effort_optimal_norm(pareto_optim_indx_Knee),safety_probability_optimal(pareto_optim_indx_Knee),200,'filled','o','MarkerFaceColor',color_Knee)
set(gca,'FontSize',fontSizeVal)
ylabel('Reach-avoid probability','interpreter','latex')
xlabel('Control effort','interpreter','latex')  %(${\Vert U \Vert}_2$)
box on
grid on
leg=legend('Optimal trade-off curve',...
           sprintf('No control effort optimization ($P$=%1.2f, $\\Vert \\rho^\\ast\\Vert_2$=%1.3f)',safety_probability_optimal(pareto_optim_indx_onlySafety), control_effort_optimal_norm(pareto_optim_indx_onlySafety)),...
           sprintf('Higher safety compromise ($P$=%1.2f, $\\Vert \\rho^\\ast\\Vert_2$=%1.3f)', safety_probability_optimal(pareto_optim_indx_Knee_safe), control_effort_optimal_norm(pareto_optim_indx_Knee_safe)),...
           sprintf('Lesser control effort compromise ($P$=%1.2f, $\\Vert \\rho^\\ast\\Vert_2$=%1.3f)', safety_probability_optimal(pareto_optim_indx_Knee), control_effort_optimal_norm(pareto_optim_indx_Knee)));
set(leg,'interpreter','latex','Location','SouthEast')
savefig(gcf,'figs\LOS_pareto.fig','compact')
saveas(gcf,'figs\LOS_pareto.png')

%% Pareto optimal curve (-log version)
figure(103)
clf
semilogy(control_effort_optimal_norm(1:end-1),-log10(safety_probability_optimal(1:end-1)),'k*--','MarkerSize',15)
hold on
plot(Polyhedron('lb',[control_effort_optimal_norm(pareto_optim_indx_onlySafety),-log10(safety_probability_optimal(pareto_optim_indx_onlySafety))],'ub',[norm(umax*ones(1,2*last_time_step)),upper_limit_y]),'color',color_onlySafety,'alpha',0.5)
plot(Polyhedron('lb',[control_effort_optimal_norm(pareto_optim_indx_Knee_safe),-log10(safety_probability_optimal(pareto_optim_indx_Knee_safe))],'ub',[norm(umax*ones(1,2*last_time_step)),upper_limit_y]),'color', color_Knee_safe,'alpha',0.5);
plot(Polyhedron('lb',[control_effort_optimal_norm(pareto_optim_indx_Knee),-log10(safety_probability_optimal(pareto_optim_indx_Knee))],'ub',[norm(umax*ones(1,2*last_time_step)),upper_limit_y]),'color', color_Knee,'alpha',0.5);
xlabel('Control effort','interpreter','latex')  %(${\Vert U \Vert}_2$)
ylabel('$-\log_{10}$(Reach-avoid prob.)','interpreter','latex')
set(gca,'FontSize',fontSizeVal)
grid on
box on
leg=legend('Optimal trade-off curve','No control effort optimization', 'Higher safety compromise', 'Lesser control effort compromise');
axis(axis_vec);
savefig(gcf,'figs\LOS_pareto_log.fig','compact')
saveas(gcf,'figs\LOS_pareto_log.png')

%% Compute time plot
figure(104)
clf
semilogx(list_of_gammas_for_pareto,elapsed_time(2:end-1),'bx','MarkerSize',20);
set(gca,'FontSize',25);
grid on;
box on;
xlabel('$\lambda$','interpreter','latex');
ylabel('Computation time (seconds)','interpreter','latex');
axis([min(list_of_gammas_for_pareto)/1.1 max(list_of_gammas_for_pareto)*1.1 0 400]);
savefig(gcf,'figs\ComputeTime.fig','compact')
saveas(gcf,'figs\ComputeTime.png')

