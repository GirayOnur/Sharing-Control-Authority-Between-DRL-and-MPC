
%% plot densities
% rho_1_1 = xx(5,  :);
% rho_2_1 = xx(12, :);
% rho_3_1 = xx(19, :);
% rho_4_1 = xx(26, :);
% rho_5_1 = xx(33, :);
% rho_6_1 = xx(40, :);
% rho_7_1 = xx(47, :);
% rho_8_1 = xx(54, :);
% rho_9_1 = xx(61, :);
% %
% %route 1:
% rho_vals = [rho_1_1,rho_2_1,rho_3_1,rho_4_1,rho_5_1,rho_6_1]';
% density_tag = zeros(length(rho_vals),1);
% timestep = zeros(length(rho_vals),1);
% k=1;
% figure(1)
% for j=1:length(rho_vals)/N
%     k_ts = 1;
%     if j == 1
%         rho_tag = 1;
%     elseif j == 2
%         rho_tag = 2;
%     elseif j == 3
%         rho_tag = 3;
%     elseif j == 4
%         rho_tag = 4;
%     elseif j == 5
%         rho_tag = 5;  
%     elseif j == 6
%         rho_tag = 6;  
%     end
%     for i=1:N
%         density_tag(k) = rho_tag;
%         timestep(k) = k_ts;
%         k=k+1;
%         k_ts=k_ts+1;
%     end
% end
% T_dens_1 = table(density_tag,rho_vals,timestep);
% h = heatmap(T_dens_1,'timestep','density_tag','ColorVariable','rho_vals');
% 
% %route 2:
% rho_vals = [rho_1_1,rho_2_1,rho_3_1,rho_7_1,rho_8_1,rho_9_1]';
% density_tag = zeros(length(rho_vals),1);
% timestep = zeros(length(rho_vals),1);
% k=1;
% figure(2)
% for j=1:length(rho_vals)/N
%     k_ts = 1;
%     if j == 1
%         rho_tag = 1;
%     elseif j == 2
%         rho_tag = 2;
%     elseif j == 3
%         rho_tag = 3;
%     elseif j == 4
%         rho_tag = 7;
%     elseif j == 5
%         rho_tag = 8;  
%     elseif j == 6
%         rho_tag = 9;  
%     end
%     for i=1:N
%         density_tag(k) = rho_tag;
%         timestep(k) = k_ts;
%         k=k+1;
%         k_ts=k_ts+1;
%     end
% end
% T_dens_2 = table(density_tag,rho_vals,timestep);
% h = heatmap(T_dens_2,'timestep','density_tag','ColorVariable','rho_vals');
% %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%% plot constraints
figure(1)
k = 0;
hold on
for i=1:N
    l_o1_j1_c1 = xx(64,i);
    l_o1_j1_c2 = xx(65,i);
    plot(k,l_o1_j1_c1+l_o1_j1_c2,'*b');
    k = k + 1;
end
%ylim([0 400]);
hold off

figure(2)
k = 0;
hold on
for i=1:N
    l_o2_j1_c1 = xx(68,i);
    l_o2_j1_c2 = xx(69,i);
    plot(k,l_o2_j1_c1+l_o2_j1_c2,'*b');
    k = k + 1;
end
%ylim([0 400]);
hold off

figure(3)
k = 0;
hold on
for i=1:N
    l_o3_j1_c1 = xx(72,i);
    l_o3_j1_c2 = xx(73,i);
    plot(k,l_o3_j1_c1+l_o3_j1_c2,'*b');
    k = k + 1;
end
%ylim([0 400]);
hold off


%Prints how far each queue went past its limit (200, 100, 100 veh).
%Queue 1 is halved because origin 1 feeds 4 lanes while the limit is set
%per 2 lanes, the same factor 2 that demando1.m applies to the demand.
%Everything above is old plotting code, left commented out.
fprintf("\n Queue 1 violation is %0.3f", (max( xx(64,:) +  xx(65,:))-200)/2);
fprintf("\n Queue 2 violation is %0.3f", (max( xx(68,:) +  xx(69,:))-100));
fprintf("\n Queue 3 violation is %0.3f", (max( xx(72,:) +  xx(73,:)) - 100));
fprintf("\n")
