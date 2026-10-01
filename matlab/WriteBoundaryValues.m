function WriteBoundaryValues()
global params_
fid = fopen('AmplInputs/BoundaryValues.txt', 'w');
fprintf(fid, '1  %.17g\r\n', params_.task.x0);
fprintf(fid, '2  %.17g\r\n', params_.task.y0);
fprintf(fid, '3  %.17g\r\n', params_.task.theta0);
fprintf(fid, '4  %.17g\r\n', params_.task.xf);
fprintf(fid, '5  %.17g\r\n', params_.task.yf);
fprintf(fid, '6  %.17g\r\n', params_.task.thetaf);
fclose(fid);
end
