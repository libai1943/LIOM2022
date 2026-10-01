function ExportReferenceData(destination)
% Export Algorithm 5-1 inputs, never an optimized answer or private source code.
global params_
if nargin < 1, destination = fullfile('..','python','data'); end
if ~isfolder(destination), mkdir(destination); end
r = CompleteInitialGuess(params_.ha_result);
names = {'x','y','theta','v','a','phy','w','xf','yf','xr','yr'};
matrix = zeros(numel(r.x),numel(names));
for ii = 1:numel(names), matrix(:,ii) = r.(names{ii})(:); end
writematrix(matrix,fullfile(destination,'reference.csv'));
writematrix(params_.scenario.dilated_map,fullfile(destination,'dilated_map.csv'));
obs = [];
for ii = 1:params_.obstacle.num_obs
    o=params_.obstacle.obs{ii}; obs=[obs;ii*ones(numel(o.x),1),o.x(:),o.y(:)];
end
writematrix(obs,fullfile(destination,'obstacles.csv'));
g=params_.vehicle; b=params_.task;
values = [params_.opti.nfe,params_.scenario.xmin,params_.scenario.xmax,params_.scenario.ymin,params_.scenario.ymax, ...
    g.lw,g.lf,g.lr,g.lb,g.vmax,g.amax,g.phymax,g.wmax, ...
    params_.opti.stc.ds,params_.opti.stc.smax,1e-5,1,1e5,4,0.01,0.01, ...
    b.x0,b.y0,b.theta0,b.xf,b.yf,r.theta(end),r.terminal_time];
writematrix(values,fullfile(destination,'parameters.csv'));
end
