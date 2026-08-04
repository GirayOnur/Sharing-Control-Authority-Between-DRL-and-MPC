function u_alinea = calc_u_alinea(x,u,pi_alinea_params,x_prev)
% PI-ALINEA ramp metering law, one controller per ramp.
%
% The metering rate is nudged from its previous value using the error
% against a target density (integral part) and how fast that density is
% changing (proportional part). The densities used are the ones right
% downstream of each ramp: x(33) for ramp 1 and x(54) for ramp 2.
% The result is clipped to [0,1]. Gains and targets come from bayesopt.

rm_1_m = max(min(u(1) + pi_alinea_params.K_I1.*(pi_alinea_params.rho_c1 -  x(33)) - pi_alinea_params.K_P1.*(x(33) - x_prev(33)),1),0);
rm_2_m = max(min(u(2) + pi_alinea_params.K_I2.*(pi_alinea_params.rho_c2 -  x(54)) - pi_alinea_params.K_P2.*(x(54) - x_prev(54)),1),0);

u_alinea = [rm_1_m;rm_2_m];

end