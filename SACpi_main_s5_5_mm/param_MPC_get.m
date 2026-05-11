function MPC_param = param_MPC_get(is_low_level)

if is_low_level == 1
    MPC_param.Np = 10;
    MPC_param.Nc = 10;
    MPC_param.M = 6;
    MPC_param.N_multi_start = 20;

else
    MPC_param.Np = 2;
    MPC_param.Nc = 2;
    MPC_param.M = 30;
    MPC_param.N_multi_start = 5;

end

MPC_param.r_cost= 0.4;
MPC_param.s_cost = 0.4;
end

