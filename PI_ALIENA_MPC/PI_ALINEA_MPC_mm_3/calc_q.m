function q = calc_q(rho_m_i,v_m_i,param)
% Flow of one vehicle class in a segment: density * speed * lanes.
% Not called anywhere - fun_benchmark_RM writes this product out inline.

q = rho_m_i*v_m_i*param.lambda;

end

