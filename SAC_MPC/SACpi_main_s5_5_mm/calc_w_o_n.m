function w_o_n = calc_w_o_n(w_o,d_o,q_o,param)
% Origin queue length update: grows with the demand at the origin and
% shrinks with the outflow that is let into the network.

w_o_n = w_o + param.T*(d_o - q_o);

end

