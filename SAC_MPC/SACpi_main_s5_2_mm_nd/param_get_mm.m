function param_esti = param_get_mm
% Perturbed parameter values used in the MPC prediction model under model
% mismatch (scenarios 3 and 4). rho_crit, rho_max, a_m and v_free differ
% from the real values; everything else matches param_get.

param_esti.rho_crit = 38.5;
param_esti.T = 10/3600;
param_esti.Q_o = 2000;
param_esti.Q_o_main = 8000;
param_esti.rho_max = 165;
param_esti.tau = 18/3600;
param_esti.L_m = 1000/1000;
param_esti.kappa = 40;
param_esti.delta = 0.0122;
param_esti.nu = 60;
param_esti.v_control_max = 200;
param_esti.v_min = 7;
param_esti.w_con = [200,100,100];
param_esti.v_con_min = 20;


param_esti.a_m.c_1 = 1.95;
param_esti.a_m.c_2 = (2.060 - 0.7*param_esti.a_m.c_1)/0.3;

param_esti.v_free.c_1 = 102;
param_esti.v_free.c_2 = (94 - 0.7*param_esti.v_free.c_1)/0.3;

param_esti.rho_crit_c_1 = 35;
param_esti.rho_crit_c_2 = (param_esti.rho_crit - 0.7*param_esti.rho_crit_c_1)/0.3;

param_esti.lambda.l1 = 4;
param_esti.lambda.l2 = 4;
param_esti.lambda.l3 = 4;
param_esti.lambda.l4 = 2;
param_esti.lambda.l5 = 2;
param_esti.lambda.l6 = 2;
param_esti.lambda.l7 = 2;
param_esti.lambda.l8 = 2;
param_esti.lambda.l9 = 2;

end

