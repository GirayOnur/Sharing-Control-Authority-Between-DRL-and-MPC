function MPC_param = param_MPC_get(is_low_level)
% MPC tuning. is_low_level = 1 returns the settings of the fast ramp
% metering controller, 0 the settings of the slow route guidance one.
%
% M is how many simulation steps one control step lasts, so with T = 10 s
% the low level acts every minute and the high level every five minutes.
% Np*M is the same for both (60 steps = 10 min), so they look equally far
% ahead. N_multi_start is how many starting points fmincon is run from.

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

MPC_param.r_cost= 0.4;  %penalty on changing the ramp rates
MPC_param.s_cost = 0.4;  %penalty on changing the split rate
end

