%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Name of the programmer: Abraham %
% Date: 2018-03-23                %
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%% Purpose
% Compute data for Figure 5 by generating pareto-optimal curves for multiple u_bounds

clear all
clc
close all
evalc('cvx_setup');
evalc('mpt_init');
fprintf('CVX and MPT3 setup done!\n\n\n')

fprintf('Creating data for Figure 5 in the NAASS paper (Subsection 4.3). (Takes about a total of 3 hours)\n')
umax_values = [0.05, 0.0625, 0.075, 0.1, 0.5];
datenowstr_values = {'0x05.mat','0x0625.mat','0x075.mat','0x1.mat','0x5.mat'};
for umax_indx = 1:length(umax_values)
    umax = umax_values(umax_indx);
    datenowstr = strcat('data/',datestr(now,'yyyymmdd'),'_pareto_',datenowstr_values{umax_indx}); 
    multicriterion_CWH
end
