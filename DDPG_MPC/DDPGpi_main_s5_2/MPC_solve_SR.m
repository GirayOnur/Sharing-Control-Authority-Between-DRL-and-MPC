function u_mpc = MPC_solve_SR(x0,u0,u_sr_prev,u_rm,k,param,MPC_param_low,MPC_param_high,scenario)
% High-level MPC controller of the hierarchical MPC framework: computes the
% vehicle split rate every T_h = 300 s over a prediction horizon of Np = 2
% high-level steps, which spans the same 600 s as the low-level horizon.
%
% In its prediction, the ramp metering rates are taken from the most recent
% low-level trajectory (u_rm), shifted one step with the terminal value
% duplicated. Solving the low-level MPC problem inside this one would be
% too expensive.

%rng('default')

Np = MPC_param_low.Np;
Nc = MPC_param_low.Nc;
M = MPC_param_low.M;

Np_high = MPC_param_high.Np;
Nc_high = MPC_param_high.Nc;
M_high = MPC_param_high.M;

r_cost = MPC_param_high.r_cost;
s_cost = MPC_param_high.s_cost;
k_opt = MPC_param_high.N_multi_start;

w_con = param.w_con;

%Optimality, step and constraint tolerances are set to 1e-2 to trade
%computational accuracy for efficiency, since this problem is solved at
%every control step of the simulation.
options = optimoptions(@fmincon,'Algorithm','sqp','Display','off', 'TolFun',1e-2, 'TolX',1e-2, 'TolCon', 1e-2, 'MaxIterations', 6);

lb = repmat([0],1,Nc_high);
ub = repmat([1],1,Nc_high);

fun = @(u_mpc) J_calc(x0, u_mpc, k, Nc, Np, M, u_sr_prev, u_rm,Nc_high, Np_high, M_high, r_cost, s_cost, param, scenario);
nonlcon = @(u_mpc) x_con(x0, u_mpc, k, w_con, Nc, Np, M,u_rm, Nc_high, Np_high, M_high, param, scenario);

%Multi-start strategy. The problem is nonconvex, so SQP is run from several
%initializations: the shifted previous solution, both bounds, then random
%guesses. The best candidate is kept.
fval = nan(1,k_opt);
u_opt = cell(1,k_opt);
feas = ones(1,k_opt);


parfor i=1:k_opt
    if i == 1
        u_init = u0;
    elseif i == 2
        u_init = lb;
    elseif i == 3
        u_init = ub;
    else
        %u_init = u0;
        u_init = repmat([rand(1)],1, Nc_high);
    end

    [u_opt{i}, fval(i), exitflag] = fmincon(fun, u_init, [], [], [], [], lb, ub, nonlcon, options);
    if exitflag < 0
        %fval(i) = inf;
        feas(i) = 0;
    end
end


%Under hard constraints the multi-start prioritizes candidates that fmincon
%reported feasible. If none is feasible, the cheapest candidate is applied
%so the simulation can carry on.
feasind = find(feas == 1);

if isempty(feasind)
    [~, i_min] = min(fval);
    u_mpc = u_opt{i_min};
else
    [~, i_min] = min(fval(feasind));
    u_mpc = u_opt{feasind(i_min)};
end

% if fval_min == inf
%     error('SQP failed to find a feasible solution.')
% else
%     u_mpc = u_opt{i_min};
% end

end


%%%
function J = J_calc(x,u,k,Nc,Np,M,u0_1,u_rm,Nc_high,Np_high,M_high,r_cost,s_cost,param,scenario)
    k_obj = k;
    u1 = [u0_1, u];
    %Quadratic penalty on fluctuations between consecutive control inputs.
    %u0_1 is the input currently applied, which is why it is put in front.
    %Quadratic penalty on fluctuations between consecutive control inputs.
    %u0_1 is the input currently applied, which is why it is put in front.
    u_diff = u1(:,2:end) - u1(:,1:end-1);  
    u_pen = s_cost.*sum(sum(u_diff.^2));
    
    u_Np = [u_rm repelem(u_rm(:, end),1,Np-Nc)];
    u_seq_low = repelem(u_Np,1,M);
    
    u_high = u;
    u_Np_high = [u_high repelem(u_high(:, end),1,Np_high-Nc_high)];
    u_seq_high = repelem(u_Np_high,1,M_high);
    
    u_seq = [u_seq_high;u_seq_low];
    %Predict the network states at every sampling step of the horizon, not just
    %at the control steps, so the objective and the constraints are evaluated
    %on the full trajectory.
    %Predict the network states at every sampling step of the horizon, not just
    %at the control steps, so the objective and the constraints are evaluated
    %on the full trajectory.
    x_seq = zeros(size(x,1),Np*M);

    for i=1:Np*M
        x = fun_benchmark_RM(x,u_seq(:,i),k_obj,param,scenario);
        x_seq(:,i) = x;
        k_obj = k_obj + 1;
    end
    

rho_1_1_c1 = x_seq(3,:);
rho_1_1_c2 = x_seq(4,:);

rho_1_2_c1 = x_seq(10,:);
rho_1_2_c2 = x_seq(11,:);

rho_1_3_c1 = x_seq(17,:);
rho_1_3_c2 = x_seq(18,:);

rho_2_1_c1 = x_seq(24,:);
rho_2_1_c2 = x_seq(25,:);

rho_3_1_c1 = x_seq(31,:);
rho_3_1_c2 = x_seq(32,:);

rho_3_2_c1 = x_seq(38,:);
rho_3_2_c2 = x_seq(39,:);

rho_4_1_c1 = x_seq(45,:);
rho_4_1_c2 = x_seq(46,:);

rho_5_1_c1 = x_seq(52,:);
rho_5_1_c2 = x_seq(53,:);

rho_5_2_c1 = x_seq(59,:);
rho_5_2_c2 = x_seq(60,:);

w_o_1_c1 = x_seq(64,:);
w_o_1_c2 = x_seq(65,:);

w_o_2_c1 = x_seq(68,:);
w_o_2_c2 = x_seq(69,:);

w_o_3_c1 = x_seq(72,:);
w_o_3_c2 = x_seq(73,:);

    
    %TTS (total time spent): vehicles on the network, density * lanes * segment
    %length, plus the vehicles waiting in the three origin queues, summed over
    %the prediction horizon and both vehicle classes.
    %TTS (total time spent): vehicles on the network, density * lanes * segment
    %length, plus the vehicles waiting in the three origin queues, summed over
    %the prediction horizon and both vehicle classes.
    tts = sum(param.T.*((rho_1_1_c1.*param.lambda.l1 + rho_1_2_c1.*param.lambda.l2 + rho_1_3_c1.*param.lambda.l3...
    + rho_2_1_c1.*param.lambda.l4 + rho_3_1_c1.*param.lambda.l5 + rho_3_2_c1.*param.lambda.l6...
    + rho_4_1_c1.*param.lambda.l7 + rho_5_1_c1.*param.lambda.l8 + rho_5_2_c1.*param.lambda.l9).*param.L_m+w_o_1_c1+w_o_2_c1+w_o_3_c1)...
+   param.T.*((rho_1_1_c2.*param.lambda.l1 + rho_1_2_c2.*param.lambda.l2 + rho_1_3_c2.*param.lambda.l3...
    + rho_2_1_c2.*param.lambda.l4 + rho_3_1_c2.*param.lambda.l5 + rho_3_2_c2.*param.lambda.l6...
    + rho_4_1_c2.*param.lambda.l7 + rho_5_1_c2.*param.lambda.l8 + rho_5_2_c2.*param.lambda.l9).*param.L_m+w_o_1_c2+w_o_2_c2+w_o_3_c2));

    %q_pen = sum(max(0,(w_o_1_c1+w_o_1_c2) - param.w_con(1)).^2 + max(0,(w_o_2_c1+w_o_2_c2) - param.w_con(2)).^2 ...
                %+ max(0,(w_o_3_c1+w_o_3_c2) - param.w_con(3)).^2);

    J = tts + u_pen*(Nc/Nc_high); %+ q_pen; %u_pen is scaled to be consistent with the high freq. action penalty in high freq. MPC (MPC_RM)
end
%%%





%%%
%Queue length limits as hard constraints: at every predicted sampling step
%the queue at each origin has to stay below w_con. fmincon gets c <= 0.
function [c,ceq] = x_con(x,u,k,w_con,Nc,Np,M,u_rm,Nc_high,Np_high,M_high,param,scenario)
    ceq = [];
    %c = [];

    k_con = k;
    u_Np = [u_rm repelem(u_rm(:, end),1,Np-Nc)];
    u_seq_low = repelem(u_Np,1,M);

    u_high = u;
    u_Np_high = [u_high repelem(u_high(:, end),1,Np_high-Nc_high)];
    u_seq_high = repelem(u_Np_high,1,M_high);

    u_seq = [u_seq_high;u_seq_low];
    x_seq = zeros(size(x,1),Np*M);
    for i=1:Np*M
        x = fun_benchmark_RM(x,u_seq(:,i),k_con,param,scenario);
        x_seq(:,i) = x;
        k_con = k_con + 1;
    end

    w_o_1_c1 = x_seq(64,:);
    w_o_1_c2 = x_seq(65,:);

    w_o_2_c1 = x_seq(68,:);
    w_o_2_c2 = x_seq(69,:);

    w_o_3_c1 = x_seq(72,:);
    w_o_3_c2 = x_seq(73,:);



    c = [(w_o_1_c1'+w_o_1_c2') - w_con(1); (w_o_2_c1'+w_o_2_c2') - w_con(2); (w_o_3_c1'+w_o_3_c2') - w_con(3)];
end
%%%