%Old check that compared two saved sets of multi-start solutions against each
%other. Needs the two .mat files below, which are not in the repo.


clear 
clc

load dingshan_sol_step_4.mat
load giray_sol_step_4.mat

u_diff = cell(1,30);
fval_diff = 999.*ones(1,30);

for i=1:30
    u_diff{i} = u_opt{i} - u_t{i};
    fval_diff(i) = fval(i) - Fval(i);
end

