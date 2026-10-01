function WriteBasicParameterFile()
global params_
fid = fopen('AmplInputs/BasicParameters.txt', 'w');
fprintf(fid, '1 %g\r\n', params_.scenario.xmin);
fprintf(fid, '2 %.17g\r\n', params_.scenario.xmax);
fprintf(fid, '3 %.17g\r\n', params_.scenario.ymin);
fprintf(fid, '4 %.17g\r\n', params_.scenario.ymax);
fprintf(fid, '5 %.17g\r\n', params_.vehicle.lw);
fprintf(fid, '6 %.17g\r\n', params_.vehicle.lf);
fprintf(fid, '7 %.17g\r\n', params_.vehicle.lr);
fprintf(fid, '8 %g\r\n', params_.vehicle.lb);
fprintf(fid, '9 %.17g\r\n', params_.vehicle.dual_disk_radius);
fprintf(fid, '10 %.17g\r\n', params_.vehicle.r2p);
fprintf(fid, '11 %.17g\r\n', params_.vehicle.f2p);
fprintf(fid, '12 %.17g\r\n', params_.vehicle.vmax);
fprintf(fid, '13 %.17g\r\n', params_.vehicle.amax);
fprintf(fid, '14 %.17g\r\n', params_.vehicle.phymax);
fprintf(fid, '15 %.17g\r\n', params_.vehicle.wmax);
fprintf(fid, '16 %.17g\r\n', params_.opti.nfe);
fprintf(fid, '17 %.17g\r\n', params_.opti.cost_function_external_penalty_weight);
fprintf(fid, '18 %.17g\r\n', params_.opti.cost_a);
fprintf(fid, '19 %.17g\r\n', params_.opti.cost_w);
fclose(fid);
end
