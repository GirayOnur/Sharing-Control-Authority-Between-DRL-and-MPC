function [demand_c1, demand_c2] = demando3mm(k,scenario)
% Demand the controller assumes in the _mm runs. Same shape as demandoN.m
% but without the sine ripple and shifted by a constant, so the controller
% predicts the wrong demand on purpose.
% k=k-1;
k=floor((k-60)/6);
switch scenario
    case 1
        t1=5;t2=t1+6;t3=t2+30;t4=t3+6; d1=300;d2=1500;
        demand=500.*(k<0)+d1.*(k>=0 & k<t1)+(d1+(d2-d1)/(t2-t1)*(k-t1)).*(k>=t1 & k<=t2)+d2.*(k>t2 & k<=t3)...
            +(d2-(d2-d1)/(t4-t3)*(k-t3)).*(k>t3 & k<=t4)+d1.*(k>t4) - 100;
    case 3
        t1=5;t2=t1+6;t3=t2+30;t4=t3+6; d1=400;d2=1300;
        demand=500.*(k<0)+d1.*(k>=0 & k<t1)+(d1+(d2-d1)/(t2-t1)*(k-t1)).*(k>=t1 & k<=t2)+d2.*(k>t2 & k<=t3)...
            +(d2-(d2-d1)/(t4-t3)*(k-t3)).*(k>t3 & k<=t4)+d1.*(k>t4) - 100;
end


c_1_ratio = 0.8;

c_2_ratio = 1 - c_1_ratio;

demand_c1 = c_1_ratio*demand;
demand_c2 = c_2_ratio*demand;

end

