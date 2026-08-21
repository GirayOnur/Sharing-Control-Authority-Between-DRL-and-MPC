function rho_m_i_n = calc_rho_m_i_n(rho_m_i, q_m_im1,q_m_i, param,l_i)
% Segment density update for one vehicle class: the inflow from the
% upstream segment minus the outflow, over the segment length and the
% number of lanes of the link.
% max(0,..) keeps the density from going negative on a bad step.

rho_m_i_n = max(0, rho_m_i + param.T/(param.L_m*param.lambda.(l_i))*(q_m_im1 - q_m_i));

end

