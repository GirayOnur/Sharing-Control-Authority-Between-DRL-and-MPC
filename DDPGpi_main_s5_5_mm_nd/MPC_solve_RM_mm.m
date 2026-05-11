function u_mpc = MPC_solve_RM_mm(x0,u0,u_rm_prev,u_sr,k,param,MPC_param_low,MPC_param_high,scenario)

%rng('default')

Np = MPC_param_low.Np;
Nc = MPC_param_low.Nc;
M = MPC_param_low.M;

Np_high = MPC_param_high.Np;
Nc_high = MPC_param_high.Nc;
M_high = MPC_param_high.M;

r_cost = MPC_param_low.r_cost;
s_cost = MPC_param_low.s_cost;
k_opt = MPC_param_low.N_multi_start;

w_con = param.w_con;

%options = optimoptions(@fmincon,'Algorithm','sqp','Display','off', 'MaxIterations', 6, 'TolFun',1e-2, 'TolX',1e-2);
options = optimoptions(@fmincon,'Algorithm','sqp','Display','off', 'TolFun',1e-2, 'TolX',1e-2, 'TolCon', 1e-2, 'MaxIterations', 6);

lb = repmat([0;0],1,Nc);
ub = repmat([1;1],1,Nc);

fun = @(u_mpc) J_calc(x0, u_mpc, k, Nc, Np, M, u_rm_prev, u_sr,Nc_high, Np_high, M_high, r_cost, s_cost, param, scenario);
nonlcon = @(u_mpc) x_con(x0, u_mpc, k, w_con, Nc, Np, M,u_sr, Nc_high, Np_high, M_high, param, scenario);

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
        u_init = repmat([rand(2,1)],1, Nc);
    end

    [u_opt{i}, fval(i), exitflag] = fmincon(fun, u_init, [], [], [], [], lb, ub, nonlcon, options);
    if exitflag < 0
        %fval(i) = inf;
        feas(i) = 0;
    end
end

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
function J = J_calc(x,u,k,Nc,Np,M,u0_1,u_sr,Nc_high,Np_high,M_high,r_cost,s_cost,param,scenario)
    k_obj = k;
    u1 = [u0_1, u];
    u_diff = u1(:,2:end) - u1(:,1:end-1);  
    u_pen = r_cost.*sum(sum(u_diff.^2));
    
    u_Np = [u repelem(u(:, end),1,Np-Nc)];
    u_seq_low = repelem(u_Np,1,M);
    
    u_high = u_sr;
    u_Np_high = [u_high repelem(u_high(:, end),1,Np_high-Nc_high)];
    u_seq_high = repelem(u_Np_high,1,M_high);
    
    u_seq = [u_seq_high;u_seq_low];
    x_seq = zeros(size(x,1),Np*M);

    for i=1:Np*M
        x = fun_benchmark_RM_mm(x,u_seq(:,i),k_obj,param,scenario);
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

    
    tts = sum(param.T.*((rho_1_1_c1.*param.lambda.l1 + rho_1_2_c1.*param.lambda.l2 + rho_1_3_c1.*param.lambda.l3...
    + rho_2_1_c1.*param.lambda.l4 + rho_3_1_c1.*param.lambda.l5 + rho_3_2_c1.*param.lambda.l6...
    + rho_4_1_c1.*param.lambda.l7 + rho_5_1_c1.*param.lambda.l8 + rho_5_2_c1.*param.lambda.l9).*param.L_m+w_o_1_c1+w_o_2_c1+w_o_3_c1)...
+   param.T.*((rho_1_1_c2.*param.lambda.l1 + rho_1_2_c2.*param.lambda.l2 + rho_1_3_c2.*param.lambda.l3...
    + rho_2_1_c2.*param.lambda.l4 + rho_3_1_c2.*param.lambda.l5 + rho_3_2_c2.*param.lambda.l6...
    + rho_4_1_c2.*param.lambda.l7 + rho_5_1_c2.*param.lambda.l8 + rho_5_2_c2.*param.lambda.l9).*param.L_m+w_o_1_c2+w_o_2_c2+w_o_3_c2));

    %q_pen = sum(max(0,(w_o_1_c1+w_o_1_c2) - param.w_con(1)).^2 + max(0,(w_o_2_c1+w_o_2_c2) - param.w_con(2)).^2 ...
                %+ max(0,(w_o_3_c1+w_o_3_c2) - param.w_con(3)).^2);

    J = tts + u_pen; %+ q_pen;
end
%%%





%%%
function [c,ceq] = x_con(x,u,k,w_con,Nc,Np,M,u_sr,Nc_high,Np_high,M_high,param,scenario)
    ceq = [];
    %c = [];

    k_con = k;
    u_Np = [u repelem(u(:, end),1,Np-Nc)];
    u_seq_low = repelem(u_Np,1,M);

    u_high = u_sr;
    u_Np_high = [u_high repelem(u_high(:, end),1,Np_high-Nc_high)];
    u_seq_high = repelem(u_Np_high,1,M_high);

    u_seq = [u_seq_high;u_seq_low];
    x_seq = zeros(size(x,1),Np*M);
    for i=1:Np*M
        x = fun_benchmark_RM_mm(x,u_seq(:,i),k_con,param,scenario);
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