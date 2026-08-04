function MPC_param = param_MPC_get(is_low_level)
% MPC settings. is_low_level = 1 returns the low-level (ramp metering)
% controller, 0 the high-level (vehicle split rate) controller.
%
% M is m_l or m_h, the number of network sampling steps in one control
% step. With T = 10 s that puts the low-level control time step at
% T_l = 60 s and the high-level one at T_h = 300 s. The prediction horizon
% Np covers the same 600 s on both levels. N_multi_start is the number of
% initializations of the multi-start strategy.

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

MPC_param.r_cost= 0.4;  %penalty weight on ramp metering rate changes
MPC_param.s_cost = 0.4;  %penalty weight on vehicle split rate changes
end

