close all; clc; clear global params_; clear all;

global params_
LoadCase();
InitializeParams();
SearchTrajectoryViaFTHA();
OptimizeTrajectoryViaLIOM();