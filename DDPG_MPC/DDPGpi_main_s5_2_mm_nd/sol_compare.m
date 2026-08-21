function [u_diff, fval_diff] = sol_compare(reference_file, candidate_file)
%SOL_COMPARE Compare two saved sets of multi-start solutions.
%   Supply the paths to MAT-files containing u_t/Fval and u_opt/fval,
%   respectively.

reference = load(reference_file);
candidate = load(candidate_file);

n_solutions = min([numel(reference.u_t), numel(reference.Fval), ...
    numel(candidate.u_opt), numel(candidate.fval)]);
u_diff = cell(1, n_solutions);
fval_diff = nan(1, n_solutions);

for i = 1:n_solutions
    u_diff{i} = candidate.u_opt{i} - reference.u_t{i};
    fval_diff(i) = candidate.fval(i) - reference.Fval(i);
end

end
