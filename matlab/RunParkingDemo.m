function result = RunParkingDemo()
root = fileparts(mfilename('fullpath'));
old_directory = pwd; restore_directory = onCleanup(@() cd(old_directory)); cd(root);
if ~isfile('AMPL.exe') || ~isfile('ipopt.exe')
    error('LIOM:MissingSolver','Install AMPL.exe and IPOPT with MA27 in this directory first. See matlab/README.md.');
end
old_rng = rng; restore_rng = onCleanup(@() rng(old_rng)); rng(0,'twister');
old_threads = getenv('OMP_NUM_THREADS'); restore_threads = onCleanup(@() setenv('OMP_NUM_THREADS',old_threads));
setenv('OMP_NUM_THREADS','1');
for name = {'AmplInputs','AmplResults'}
    if ~isfolder(name{1}), mkdir(name{1}); end
end
fid = fopen(fullfile('AmplResults','solver.log'),'w'); fclose(fid);
global params_
params_ = struct();
LoadCase(); InitializeParams();
fprintf('\nLIOM 2022 / Chapter 5 -- Case 12, 200 configuration points\n');
fprintf('Searching for the protected Hybrid A* reference trajectory...\n');
SearchTrajectoryViaFTHA();
if isempty(params_.ha_result.x), error('LIOM:SearchFailed','No reference trajectory was found.'); end
result = OptimizeTrajectoryViaLIOM();
DrawResults(result);
fprintf('Converged after %d iterations: J = %.7f, T = %.7f s.\n',size(result.iterations,1),result.cost,result.terminal_time);
end
