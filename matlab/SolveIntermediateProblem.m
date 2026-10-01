function r = SolveIntermediateProblem()
% Clear only this program's own output before running the AMPL command file.
names = {'x','y','theta','v','a','phy','w','xf','yf','xr','yr', ...
    'terminal_time','cost','infeasibility','solve_result_num'};
for ii = 1 : numel(names)
    file = fullfile('AmplResults',[names{ii},'.txt']);
    if isfile(file), delete(file); end
end
[status,output] = system('".\AMPL.exe" SolveIntermediate.run 2>&1');
fid = fopen(fullfile('AmplResults','solver.log'),'a');
if fid >= 0, fprintf(fid,'%s\n',output); fclose(fid); end
if status ~= 0, error('LIOM:SolverProcess','AMPL failed. See AmplResults/solver.log.\n%s',output); end
r = LoadAmplSolution();
end
