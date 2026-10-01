function DrawParkingScenario()
% Draw the original Chapter 5 obstacles in the current axes.
global params_
obstacle_vertexes_ = params_.obstacle.obs;
environment_scale_ = struct('environment_x_min',params_.scenario.xmin,'environment_x_max',params_.scenario.xmax,'environment_y_min',params_.scenario.ymin,'environment_y_max',params_.scenario.ymax);
hold on;
for ii = 1 : numel(obstacle_vertexes_)
    fill(obstacle_vertexes_{ii}.x,obstacle_vertexes_{ii}.y,[0.43,0.46,0.49], ...
        'EdgeColor',[0.30,0.33,0.36],'HandleVisibility','off');
end
axis equal; box on; grid on;
axis([environment_scale_.environment_x_min,environment_scale_.environment_x_max, ...
    environment_scale_.environment_y_min,environment_scale_.environment_y_max]);
xlabel('x / m'); ylabel('y / m');
set(gca,'FontSize',12,'Layer','top','GridAlpha',0.13);
end
