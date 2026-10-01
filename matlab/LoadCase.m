function LoadCase()
global params_
load([pwd, '\ParkingBenchmarks\CaseNo_', num2str(12), '.mat']);
params_.task.x0 = x0;
params_.task.y0 = y0;
params_.task.theta0 = theta0;
params_.task.xf = xtf;
params_.task.yf = ytf;
params_.task.thetaf = thetatf;
params_.obstacle.num_obs = length(obstacles);
params_.obstacle.obs = obstacles;
end