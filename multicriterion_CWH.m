%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Name of the programmer: Abraham %
% Date: 2018-03-23                %
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%% Purpose
% Generates the data for the plots in the NAASS 2018 paper Subsection 4.2

%% Notes
% - Run FigureSubSect42.m to obtain the plots
% - Generates two plots for visual confirmation but run the above script for the plots in the paper
% - Uses the problem parameters (dynamics, safe set, target set, etc) defined in params.m

%% Problem parameters
if ~exist('umax','var')
    clear all
    clc
    close all
    evalc('cvx_setup');
    evalc('mpt_init');
    fprintf('CVX and MPT3 setup done!\n\n\n')
    
    datenowstr = strcat('data/',datestr(now,'yyyymmdd'),'_pareto_0x1.mat');
    umax=0.1;
    flag_single_run = 1;
    fprintf('Creating data for Figures 1--4 in the NAASS paper (Subsection 4.2). (Takes about 40 minutes)\n')
    fprintf('umax = %1.4f\n', umax)
else
    % Part of scan_through_umax.m (datenowstr and umax will be provided)
    flag_single_run = 0;
    fprintf('Stop now and do ''clear;multicriterion_CWH;'' if you want data for Figures 1--4.\n\n')
    fprintf('umax = %1.4f\n', umax)
end
if ~exist('time_horizon','var')
    params
    fprintf('Using params.m for parameters\n')
end

%% Time convention
% Evolution starts from 0 and ends at time_horizon
% Hence, the total number of timesteps is time_horizon+1
% To make this clear, I will use last_time_step to refer the last time
% state is known 
last_time_step = time_horizon;

%% Pareto optimality
total_no_of_gammas = no_of_gammas + 2;
list_of_gammas_for_pareto = logspace( -0.2, 2, no_of_gammas );
control_effort_optimal_norm = zeros(1,total_no_of_gammas);
control_effort_optimal = zeros(input_dimension*last_time_step,total_no_of_gammas);
safety_probability_optimal = zeros(1,total_no_of_gammas);
elapsed_time = zeros(1,total_no_of_gammas);

%% Compute concatenated matrices for state, input, and disturbance
%[x_0; x_1;..., x_T]=concatenated_A_matrix * x_0 + H_matrix * U + G_matrix * W;
[concatenated_A_matrix, H_matrix, G_matrix] = getConcatenatedMatrices(system_matrix, input_matrix, disturbance_matrix, last_time_step);
H_matrix_no_initial_state=H_matrix(state_dimension+1:end,:);

%% Create the reach-avoid open-loop cost function
% Reach-avoid tube constraint
% Stay in the safe set for t=1,...,T-1 and Reach the target set at T
% Safety constraint at t=0 is assumed to hold (TO BE ADDED IN CONSTRAINTS)
reachAvoidTube_A=blkdiag(kron(eye(last_time_step-1),A_safe_set),A_target_set);
reachAvoidTube_b=[kron(ones(last_time_step-1,1),b_safe_set);
                  b_target_set];
% U^N constraint
%A_inequalities_input = kron(eye(last_time_step),A_input_set);
%b_inequalities_input = kron(ones(last_time_step,1),b_input_set);
lower_bound_input_vec=-umax*ones(input_dimension*last_time_step,1);
upper_bound_input_vec=umax*ones(input_dimension*last_time_step,1);

% Stochastics of G*W (initial state affects only the mean not the Sigma)
concatenated_disturbance_mean=kron(ones(last_time_step,1),mean_vector);
concatenated_disturbance_sigma=kron(eye(last_time_step),sigma_matrix);
concatenated_state_mean_no_input_and_no_initial_state_effect = G_matrix*concatenated_disturbance_mean;
concatenated_state_sigma_no_input=G_matrix*concatenated_disturbance_sigma*G_matrix';

negative_log_safety_cost_function = @(U) -log(reachAvoidCostFunctionAssumingValidInitialState_CWH(initial_state,...
                                  U,...
                                  concatenated_A_matrix,...
                                  concatenated_state_mean_no_input_and_no_initial_state_effect,...
                                  concatenated_state_sigma_no_input,...
                                  H_matrix_no_initial_state,...
                                  reachAvoidTube_A,...
                                  reachAvoidTube_b,...
                                  state_dimension,...
                                  myeps));

fprintf('gamma varied from 0 to Inf in %d ''steps''. Higher gamma implies higher priority to efficiency.\n\n\n', total_no_of_gammas);

%% Analyze the case where \lambda = \infty
timer_val=tic;
optimal_input_vector_for_no_safety = zeros(input_dimension*last_time_step,1);
safety_probability_optimal(end) = exp(-negative_log_safety_cost_function(optimal_input_vector_for_no_safety)); control_effort_optimal_norm(end) = norm(optimal_input_vector_for_no_safety);
control_effort_optimal(:,end) = optimal_input_vector_for_no_safety;
elapsed_time(end)=toc(timer_val);
fprintf('Analyzing gamma: Inf (%d/%d) [No safety requirement] | Completed\n', total_no_of_gammas, total_no_of_gammas);
fprintf('Optimal value --- Control effort: %1.2f, Safety probability: %1.2f\n\n',control_effort_optimal_norm(end), safety_probability_optimal(end));

%% Get initial input vector solution by solving (18) in the NAASS paper
%initial_input_vector: Optimize such that the mean state is within the safe set and the target set
cvx_begin quiet
    variable initial_input_vector(input_dimension*last_time_step)
    minimize (norm(initial_input_vector))
    subject to
    reachAvoidTube_A*(concatenated_A_matrix(5:end,:)*initial_state + H_matrix_no_initial_state*initial_input_vector)<= reachAvoidTube_b 
    -umax <= initial_input_vector <= umax 
cvx_end
initial_guess_for_input_vector = initial_input_vector;
%fprintf('Initialization control input\n')
%disp(initial_guess_for_input_vector')
assert(~any(isnan(initial_guess_for_input_vector)),'Infeasible construction for initialization of (16)');


PSoptions = psoptimset('Display',display_string,'TolMesh',mesh_tolerance_for_patternsearch,'MaxFunEvals',MaxFunEvals); %'TolCon',constraint_tolerance_for_patternsearch,

%% Patternsearch and cvx to compute the pareto optimal tradeoff curve
for gamma_indx=total_no_of_gammas-1:-1:2
    timer_val=tic;
    gamma_value = list_of_gammas_for_pareto(gamma_indx-1);
    fprintf('Analyzing gamma: %4.4f (%d/%d) | ', gamma_value, gamma_indx, total_no_of_gammas);
    cost_function = @(U)  gamma_value*norm(U) + negative_log_safety_cost_function(U);
    [U_star,optimal_negative_log_reachAvoid_value] = patternsearch(cost_function,...
                                                                   initial_guess_for_input_vector,...
                                                                   [],[],[],[],...
                                                                   lower_bound_input_vec,...
                                                                   upper_bound_input_vec,...
                                                                   [],...
                                                                   PSoptions);
    safety_probability_optimal(gamma_indx) = exp(-negative_log_safety_cost_function(U_star));
    control_effort_optimal_norm(gamma_indx) = norm(U_star);
    control_effort_optimal(:,gamma_indx) = U_star;
    elapsed_time(gamma_indx)=toc(timer_val);
    fprintf('Completed\n');
    fprintf('Optimal value --- Control effort: %1.2f, Safety probability: %1.2f\n\n',control_effort_optimal_norm(gamma_indx), safety_probability_optimal(gamma_indx));
end

%% Analyze the case where \lambda = 0
timer_val=tic;
fprintf('Analyzing gamma: 0 (%d/%d) [No efficiency requirements] | ', 1, total_no_of_gammas);
[U_star,optimal_negative_log_reachAvoid_value] = patternsearch(negative_log_safety_cost_function,...
                                                               initial_guess_for_input_vector,...
                                                               [],[],[],[],...
                                                               lower_bound_input_vec,...
                                                               upper_bound_input_vec,...
                                                               [],...
                                                               PSoptions);
safety_probability_optimal(1) = exp(-negative_log_safety_cost_function(U_star));
control_effort_optimal_norm(1) = norm(U_star);
control_effort_optimal(:,1) = U_star;
elapsed_time(1)=toc(timer_val);
fprintf('Completed\n');
fprintf('Optimal value --- Control effort: %1.2f, Safety probability: %1.2f\n\n',control_effort_optimal_norm(1), safety_probability_optimal(1));
%fprintf('Maximum of u: %1.2f, Constraint satisfaction: %d\n',umax,any(any(abs(control_effort_optimal)>umax)))

%% Pareto optimal curve visual confirmation
figure(102)
clf
plot(control_effort_optimal_norm(2:end),safety_probability_optimal(2:end),'bs--','MarkerSize',10)
xlabel('Control effort (${\Vert U \Vert}_2$)','interpreter','latex')
ylabel('Reach-avoid probability')
set(gca,'FontSize',25)
grid on
box on
title('Pareto optimal curve');

%% Initial solution visual confirmation
figure(101);
clf
box on;
hold on;
plot(polytope_target_set.slice([3,4], slice_at_vx_vy), 'color', 'k')
plot(polytope_safe_set.slice([3,4], slice_at_vx_vy), 'color', 'y')
initial_guess_for_mean_vector_for_plot_vec= concatenated_A_matrix(5:end,:)*initial_state + H_matrix_no_initial_state*initial_guess_for_input_vector;
initial_guess_for_mean_vector_for_plot=reshape(initial_guess_for_mean_vector_for_plot_vec,4,[]);
scatter(initial_guess_for_mean_vector_for_plot(1,:),initial_guess_for_mean_vector_for_plot(2,:),100,'ko');
hold on
scatter(initial_state(1),initial_state(2),100,'ks');
scatter(initial_state(1),initial_state(2),100,'kx');
title('Initialization trajectory for the patternsearch');

save(datenowstr);
