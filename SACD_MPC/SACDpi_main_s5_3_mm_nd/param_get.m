function param = param_get
% Real values of the multi-class METANET parameters. These drive the
% simulator. Under model mismatch the MPC prediction model uses the
% perturbed values from param_get_mm instead.

param.rho_crit = 33.5;
param.T = 10/3600;  %network sampling time step, 10 s written in hours
param.Q_o = 2000;
param.Q_o_main = 8000;
param.rho_max = 180;
param.tau = 18/3600;
param.L_m = 1000/1000;
param.kappa = 40;
param.delta = 0.0122;
param.nu = 60;
param.v_control_max = 200;
param.v_min = 7;
param.w_con = [200,100,100];  %queue length limits at O1, O2, O3 [veh]
param.v_con_min = 20;


%Two vehicle classes, class 1 being the faster one. The class 2 values are
%set so that a 70/30 mix of the classes reproduces the single-class values
%(a_m = 1.867, v_free = 102 km/h), giving a_m,2 = 2.023 and
%v_free,2 = 83.33 km/h.
param.a_m.c_1 = 1.8;
param.a_m.c_2 = (1.867 - 0.7*param.a_m.c_1)/0.3;

param.v_free.c_1 = 110; 
param.v_free.c_2 = (102 - 0.7*param.v_free.c_1)/0.3;

param.rho_crit_c_1 = 30;
param.rho_crit_c_2 = (param.rho_crit - 0.7*param.rho_crit_c_1)/0.3;

%Lanes per segment. l1-l3 are the three segments of the mainstream link,
%l4-l6 the primary route and l7-l9 the secondary route.
param.lambda.l1 = 4;
param.lambda.l2 = 4;
param.lambda.l3 = 4;
param.lambda.l4 = 2;
param.lambda.l5 = 2;
param.lambda.l6 = 2;
param.lambda.l7 = 2;
param.lambda.l8 = 2;
param.lambda.l9 = 2;

end

