
%% run bayesian optimization to tune parameters of PI-ALINEA
clear
clc

rng(3)

var1 = optimizableVariable(K_P1=[0,15]);
var2 = optimizableVariable(K_P2=[0,15]);
var3 = optimizableVariable(K_I1=[0,4]);
var4 = optimizableVariable(K_I2=[0,4]);
var5 = optimizableVariable(rho_c1=[30,100]);
var6 = optimizableVariable(rho_c2=[30,100]);

optimVars = [var1,var2,var3,var4,var5,var6];

objFun = @(params) bayes_cost_PI_ALINEA_MPC_mm(params);

optimResults = bayesopt(objFun,optimVars,'MaxObjectiveEvaluations', inf, 'MaxTime', 14400 , UseParallel=true, PlotFcn=[], Verbose=0);

opt_result_doc_name = 'Bayes_opt_PI_ALINEA_mm_result_' + string(datetime('now'), 'yyyy-MM-dd hh_mm_ss') + '.mat';
save(opt_result_doc_name)

%% run the experiment using the tuned parameters
optimVals = bestPoint(optimResults);
pi_alinea_params.K_P1 = optimVals{1,1};
pi_alinea_params.K_P2 = optimVals{1,2};
pi_alinea_params.K_I1 = optimVals{1,3};
pi_alinea_params.K_I2 = optimVals{1,4};
pi_alinea_params.rho_c1 = optimVals{1,5};
pi_alinea_params.rho_c2 = optimVals{1,6};


run benchmark_PI_ALINEA_MPC_SR_RM_mm.m
