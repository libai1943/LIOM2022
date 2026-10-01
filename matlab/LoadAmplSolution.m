function r = LoadAmplSolution()
global params_
file = fullfile('AmplResults','solve_result_num.txt');
if ~isfile(file), error('LIOM:MissingStatus','Missing solver status; see AmplResults/solver.log.'); end
status = load(file);
if ~isscalar(status) || ~isfinite(status) || status < 0 || status >= 100
    error('LIOM:SolverFailed','Unsuccessful IPOPT status: %s. See solver.log.',mat2str(status));
end
r.solve_result_num = status;
names = {'x','y','theta','v','a','phy','w','xf','yf','xr','yr','terminal_time','cost','infeasibility'};
for ii = 1 : numel(names)
    file = fullfile('AmplResults',[names{ii},'.txt']);
    if ~isfile(file), error('LIOM:MissingOutput','Missing %s.',file); end
    data = load(file); expected = params_.opti.nfe;
    if ii > 11, expected = 1; end
    if numel(data) ~= expected || any(~isfinite(data(:)))
        error('LIOM:InvalidOutput','Invalid %s.',file);
    end
    r.(names{ii}) = data(:)';
end
end
