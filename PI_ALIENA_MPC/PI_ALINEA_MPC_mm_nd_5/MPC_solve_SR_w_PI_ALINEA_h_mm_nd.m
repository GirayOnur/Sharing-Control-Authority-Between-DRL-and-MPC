function u_mpc = MPC_solve_SR_w_PI_ALINEA_h_mm_nd(x0,u0,u_mpc_prev,u_rm,k,param,MPC_param_low,MPC_param_high,scenario,pi_alinea_params,x_prev,u_PI_ALINEA)
% High level MPC (split rate) for the PI-ALINEA setup. Same idea as the DRL
% version: inside the prediction the PI-ALINEA law is stepped every M steps,
% so the MPC accounts for how the ramp controller will react.
% The _h variant puts the queue limits in as hard constraints, the other one
% as a penalty in the cost.

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

%options = optimoptions(@fmincon,'Algorithm','sqp','Display','off','TolFun',1, 'TolX',1e-2, 'TolCon', 1e-2);
%Loose tolerances and a small iteration cap on purpose: this solve happens
%hundreds of times per experiment, so accuracy is traded for speed.
options = optimoptions(@fmincon,'Algorithm','sqp','Display','off','OptimalityTolerance',1e-2, 'StepTolerance',1e-2, 'TolCon', 1e-2, 'MaxIterations', 6);


lb = repmat([0],1,Nc_high);
ub = repmat([1],1,Nc_high);

fun = @(u_mpc) J_calc(x0, u_mpc, k, Nc, Np, M, u_mpc_prev, u_rm,Nc_high, Np_high, M_high, r_cost, s_cost, param, scenario,pi_alinea_params,x_prev,u_PI_ALINEA);
%nonlcon = [];
nonlcon = @(u_mpc) x_con(x0, u_mpc, k, w_con, Nc, Np, M,u_rm, Nc_high, Np_high, M_high, param, scenario,pi_alinea_params,x_prev,u_PI_ALINEA);

%Multi-start. SQP only finds a local minimum, so the problem is solved from
%several starting points: the shifted previous solution, both bounds, then
%random guesses. The best of them is used.
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

%Prefer a solution fmincon called feasible. If none of them is, fall back to
%the cheapest one anyway so the simulation can carry on.
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
function J = J_calc(x,u,k,Nc,Np,M,u0_1,u_rm,Nc_high,Np_high,M_high,r_cost,s_cost,param,scenario,pi_alinea_params,x_prev,u_PI_ALINEA)
    k_obj = k;
    u1 = [u0_1, u];
    %Penalty on changing the control action, so it does not jump around.
    %u0_1 is the action currently applied, which is why it is put in front.
    %Penalty on changing the control action, so it does not jump around.
    %u0_1 is the action currently applied, which is why it is put in front.
    u_diff = u1(:,2:end) - u1(:,1:end-1);  
    u_pen = s_cost.*sum(sum(u_diff.^2));
    
    
    u_high = u;
    u_Np_high = [u_high repelem(u_high(:, end),1,Np_high-Nc_high)];
    u_seq_high = repelem(u_Np_high,1,M_high);
    
    %Roll the network forward over the whole prediction horizon and keep every
    %state, since the cost and the constraints are evaluated over all of them.
    %Roll the network forward over the whole prediction horizon and keep every
    %state, since the cost and the constraints are evaluated over all of them.
    x_seq = zeros(size(x,1),Np*M);

    for i=1:Np*M
        if mod(k_obj,M) == 0    
            u_PI_ALINEA = calc_u_alinea(x,u_PI_ALINEA,pi_alinea_params,x_prev);
            x_prev = x;
        end

        x = fun_benchmark_RM_mm(x,[u_seq_high(:,i);u_PI_ALINEA],k_obj,param,scenario);
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


    %TTS (total time spent): vehicles on the road, density * lanes * length,
    %plus the vehicles waiting in the three origin queues, summed over the
    %horizon and over both classes. This is what the controllers minimise.
    %TTS (total time spent): vehicles on the road, density * lanes * length,
    %plus the vehicles waiting in the three origin queues, summed over the
    %horizon and over both classes. This is what the controllers minimise.
    tts = sum(param.T.*((rho_1_1_c1.*param.lambda.l1 + rho_1_2_c1.*param.lambda.l2 + rho_1_3_c1.*param.lambda.l3...
    + rho_2_1_c1.*param.lambda.l4 + rho_3_1_c1.*param.lambda.l5 + rho_3_2_c1.*param.lambda.l6...
    + rho_4_1_c1.*param.lambda.l7 + rho_5_1_c1.*param.lambda.l8 + rho_5_2_c1.*param.lambda.l9).*param.L_m+w_o_1_c1+w_o_2_c1+w_o_3_c1)...
+   param.T.*((rho_1_1_c2.*param.lambda.l1 + rho_1_2_c2.*param.lambda.l2 + rho_1_3_c2.*param.lambda.l3...
    + rho_2_1_c2.*param.lambda.l4 + rho_3_1_c2.*param.lambda.l5 + rho_3_2_c2.*param.lambda.l6...
    + rho_4_1_c2.*param.lambda.l7 + rho_5_1_c2.*param.lambda.l8 + rho_5_2_c2.*param.lambda.l9).*param.L_m+w_o_1_c2+w_o_2_c2+w_o_3_c2));

    %q_pen = sum(max(0,(w_o_1_c1+w_o_1_c2) - param.w_con(1)).^2 + max(0,(w_o_2_c1+w_o_2_c2) - param.w_con(2)).^2 ...
                %+ max(0,(w_o_3_c1+w_o_3_c2) - param.w_con(3)).^2);

    J = tts + u_pen*(Nc/Nc_high); %+ q_pen; %  + u_pen_RL; %u_pen is scaled to be consistent with the high freq. action penalty in high freq. MPC (MPC_RM)
end
%%%





%%%
%Hard constraint: at every predicted step the queue at each origin has to
%stay below w_con. fmincon gets it as c <= 0.
function [c,ceq] = x_con(x,u,k,w_con,Nc,Np,M,u_rm,Nc_high,Np_high,M_high,param,scenario,pi_alinea_params,x_prev,u_PI_ALINEA)
    ceq = [];
    %c = [];

    k_con = k;

    u_high = u;
    u_Np_high = [u_high repelem(u_high(:, end),1,Np_high-Nc_high)];
    u_seq_high = repelem(u_Np_high,1,M_high);

    x_seq = zeros(size(x,1),Np*M);
    for i=1:Np*M

        if mod(k_con,M) == 0     
            u_PI_ALINEA = calc_u_alinea(x,u_PI_ALINEA,pi_alinea_params,x_prev);
            x_prev = x;
        end

        x = fun_benchmark_RM_mm(x,[u_seq_high(:,i);u_PI_ALINEA],k_con,param,scenario);
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