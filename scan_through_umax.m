%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Name of the programmer: Abraham %
% Date: 2018-03-23                %
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%% Purpose
% Compute data for Figure 5 by generating pareto-optimal curves for multiple u_bounds

clear
clc
close all
fprintf('Creating data for Figure 5 in the NAASS paper (Subsection 4.3). (Takes about a total of 3 hours)\n')
umax_values = [0.05, 0.0625, 0.075, 0.1, 0.5];
for umax = umax_values
    multicriterion_CWH
end
