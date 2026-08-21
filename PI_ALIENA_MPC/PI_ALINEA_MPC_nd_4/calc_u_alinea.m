function u_alinea = calc_u_alinea(x,u,pi_alinea_params,x_prev)
% PI-ALINEA state-feedback ramp metering law, one controller per on-ramp.
%
% The metering rate is updated from its previous value with proportional
% and integral feedback of the measured bottleneck density:
%   u(k+1) = u(k) + K_R*(rho_bar - rho_b(k)) - K_A*(rho_b(k) - rho_b(k-1))
% The fields below are named after the feedback terms rather than the
% paper's gains, so K_I here is K_R and K_P here is K_A. The bottleneck
% densities are the ones just downstream of each ramp, x(33) for on-ramp O2
% and x(54) for on-ramp O3. The result is clipped to [0,1], and the gains
% and desired densities come from the Bayesian optimization run.

rm_1_m = max(min(u(1) + pi_alinea_params.K_I1.*(pi_alinea_params.rho_c1 -  x(33)) - pi_alinea_params.K_P1.*(x(33) - x_prev(33)),1),0);
rm_2_m = max(min(u(2) + pi_alinea_params.K_I2.*(pi_alinea_params.rho_c2 -  x(54)) - pi_alinea_params.K_P2.*(x(54) - x_prev(54)),1),0);

u_alinea = [rm_1_m;rm_2_m];

end