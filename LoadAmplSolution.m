function [x, y, theta, v, a, phy, w, terminal_time, infeasibility] = LoadAmplSolution()
load([pwd, '\AmplResults\', 'x.txt']);
load([pwd, '\AmplResults\', 'y.txt']);
load([pwd, '\AmplResults\', 'theta.txt']);
load([pwd, '\AmplResults\', 'v.txt']);
load([pwd, '\AmplResults\', 'a.txt']);
load([pwd, '\AmplResults\', 'phy.txt']);
load([pwd, '\AmplResults\', 'w.txt']);
load([pwd, '\AmplResults\', 'terminal_time.txt']);
load([pwd, '\AmplResults\', 'infeasibility.txt']);
end