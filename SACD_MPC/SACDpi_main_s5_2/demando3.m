function [demand_c1, demand_c2] = demando3(k,scenario)
% Demand at this origin over time, split into the two vehicle classes.
% k is the simulation step and is first mapped onto the demand time scale:
% the first 60 steps are the warm-up (k<0 below) and after that the demand
% changes once every 6 steps, i.e. every minute.
% k=k-1;
k=floor((k-60)/6);
switch scenario
    case 1
        t1=5;t2=t1+6;t3=t2+30;t4=t3+6; d1=300;d2=1500;
        demand=500.*(k<0)+d1.*(k>=0 & k<t1)+(d1+(d2-d1)/(t2-t1)*(k-t1)).*(k>=t1 & k<=t2)+d2.*(k>t2 & k<=t3)...
            +(d2-(d2-d1)/(t4-t3)*(k-t3)).*(k>t3 & k<=t4)+d1.*(k>t4) + 200*sin(0.02*(k-60)).*((k-60)>=0);
    case 3
        t1=5;t2=t1+6;t3=t2+30;t4=t3+6; d1=400;d2=1300;
        demand=500.*(k<0)+d1.*(k>=0 & k<t1)+(d1+(d2-d1)/(t2-t1)*(k-t1)).*(k>=t1 & k<=t2)+d2.*(k>t2 & k<=t3)...
            +(d2-(d2-d1)/(t4-t3)*(k-t3)).*(k>t3 & k<=t4)+d1.*(k>t4) + 200*sin(0.02*(k-60)).*((k-60)>=0);
end


c_1_ratio = 0.8; %share of the demand that is class 1

c_2_ratio = 1 - c_1_ratio;

demand_c1 = c_1_ratio*demand;
demand_c2 = c_2_ratio*demand;

end

