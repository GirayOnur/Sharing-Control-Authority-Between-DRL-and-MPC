%Compares one control input over time between two saved runs, by default an
%RL-MPC run against a hierarchical MPC one. Set the file names to runs that
%exist in this folder.

uind = 2; %1 -> SR, 2 -> RM1, 3 -> RM2

load('RL_MPC_SR_RM_result_2025-06-12 11_31_32.mat')

plot(uu(uind,:), 'r')
hold on

load('MPC_RM_SR_hier_result_2025-06-06 12_57_32.mat')

plot(uu(uind,:), 'b')

ylim([0,1])