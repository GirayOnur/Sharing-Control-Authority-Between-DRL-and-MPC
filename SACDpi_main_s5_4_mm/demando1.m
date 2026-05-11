function [demand_c1, demand_c2] = demando1(k,scenario)
%DEMANDO1 Summary of this function goes here
%   Detailed explanation goes here
% k=k-1;
k=floor((k-60)/6);
switch scenario
    case 1
        t1=105;t2=130; d0=3000; d1=3500; d2=1040;
        demand=d0.*(k<0)+d1.*(k>=0 & k<=t1)+(d1-(d1-d2)/(t2-t1)*(k-t1)).*(k>t1 & k<=t2)+d2.*(k>t2)+400*sin(0.03*(k-60)).*((k-60)>=0);
    case 3
        t1=105;t2=130; d0=3000; d1=3500; d2=1040;
        demand=d0.*(k<0)+d1.*(k>=0 & k<=t1)+(d1-(d1-d2)/(t2-t1)*(k-t1)).*(k>t1 & k<=t2)+d2.*(k>t2)+400*sin(0.03*(k-60)).*((k-60)>=0);
end

c_1_ratio = 0.8;

c_2_ratio = 1 - c_1_ratio;
demand_scale = 0.95; %to let MPC find feasible solution for the queue constraints
demand_c1 = c_1_ratio*2*demand*demand_scale; %multiplied by 2 since the number of lanes is 4
demand_c2 = c_2_ratio*2*demand*demand_scale; %multiplied by 2 since the number of lanes is 4

end

