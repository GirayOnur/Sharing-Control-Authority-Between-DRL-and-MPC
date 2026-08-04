function V_tilde_rho_m_i = calc_V_tilde_rho_m_i(rho_m_i,theta_m_i_c1,theta_m_i_c2,v_control,param,c_i)
% Desired speed once both classes share the road. A class cannot drive
% faster than the mix around it allows, so the second term blends the two
% class speeds using theta, the share each class has of this segment.

V_tilde_rho_m_i = min( calc_V_rho_m_i(rho_m_i,v_control,param,c_i),...
    (theta_m_i_c1*calc_V_rho_m_i(rho_m_i,v_control,param,'c_1') + theta_m_i_c2*calc_V_rho_m_i(rho_m_i,v_control,param,'c_2')) );


end

