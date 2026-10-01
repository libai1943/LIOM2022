function DrawResults(r)
% Two static figures only: spatial result and state/control profiles.
global params_
vehicle_kinematics_ = struct('vehicle_v_max',params_.vehicle.vmax,'vehicle_phy_max',params_.vehicle.phymax,'vehicle_a_max',params_.vehicle.amax,'vehicle_w_max',params_.vehicle.wmax);
delete(findall(0,'Type','figure','Tag','LIOM2022Trajectory'));
delete(findall(0,'Type','figure','Tag','LIOM2022Profiles'));
figure('Color','w','Name','Chapter 5 - trajectory and footprints', ...
    'NumberTitle','off','Tag','LIOM2022Trajectory','Position',[100,90,900,800]);
DrawParkingScenario();
h_footprints = DrawTrajFootprints(r.x,r.y,r.theta);
p = [r.initial_guess.x(:),r.initial_guess.y(:),r.initial_guess.theta(:)];
for jj = 1 : numel(r.history)-1
    h_intermediate = plot(r.history{jj}.x,r.history{jj}.y,'Color',[0.65,0.65,0.65],'LineWidth',0.9,'HandleVisibility','off');
end
h_initial = plot(p(:,1),p(:,2),'--','Color',[0.15,0.17,0.20], ...
    'LineWidth',1.6,'DisplayName','Hybrid A* reference');
h_optimal = plot(r.x,r.y,'Color',[0.84,0.18,0.13], ...
    'LineWidth',2.2,'DisplayName','Optimized trajectory');
for ii = [1,numel(r.x)]
    V = CreateVehiclePolygon(r.x(ii),r.y(ii),r.theta(ii),2); V = [V.x(:),V.y(:)];
    plot(V(:,1),V(:,2),'Color',[0.10,0.39,0.64],'LineWidth',1.3,'HandleVisibility','off');
end
quiver(r.x([1,end]),r.y([1,end]),1.8*cos(r.theta([1,end])),1.8*sin(r.theta([1,end])),0, ...
    'Color',[0.10,0.22,0.32],'LineWidth',1.2,'MaxHeadSize',0.6,'HandleVisibility','off');
text(r.x(1)+0.8,r.y(1)-1,'Start','FontSize',11,'FontWeight','bold');
text(r.x(end)-1.5,r.y(end)+2.3,'Goal','FontSize',11,'FontWeight','bold');
title(sprintf('LIOM parking | J = %.4f | %d iterations',r.cost,numel(r.history)),'FontSize',15);
legend([h_optimal,h_initial,h_footprints],'Location','northoutside','Orientation','horizontal');

figure('Color','w','Name','Chapter 5 - optimal states and controls', ...
    'NumberTitle','off','Tag','LIOM2022Profiles','Position',[150,100,1200,760]);
tiledlayout(2,3,'TileSpacing','compact','Padding','compact');
t = linspace(0,r.terminal_time,numel(r.x));
names = {'v','phy','theta','a','w'};
labels = {'v / (m s^{-1})','\phi / rad','\theta / rad','a / (m s^{-2})','\omega / (rad s^{-1})'};
titles = {'Speed','Steering angle','Heading','Acceleration','Steering rate'};
limits = [vehicle_kinematics_.vehicle_v_max,vehicle_kinematics_.vehicle_phy_max, ...
    NaN,vehicle_kinematics_.vehicle_a_max,vehicle_kinematics_.vehicle_w_max];
for ii = 1 : 5
    nexttile(ii);
    if ii <= 3
        plot(t,r.(names{ii}),'Color',[0.10,0.39,0.64],'LineWidth',1.7);
    else
        stairs(t,r.(names{ii}),'Color',[0.84,0.18,0.13],'LineWidth',1.5);
    end
    if isfinite(limits(ii))
        yline(limits(ii),':','Color',[0.55,0.55,0.55]);
        yline(-limits(ii),':','Color',[0.55,0.55,0.55]);
        ylim(1.12*[-limits(ii),limits(ii)]);
    end
    grid on; box on; xlim([0,r.terminal_time]);
    xlabel('t / s'); ylabel(labels{ii}); title(titles{ii}); set(gca,'FontSize',11);
end
nexttile(6); plot(t,r.x,'LineWidth',1.7); hold on; plot(t,r.y,'LineWidth',1.7);
grid on; box on; xlim([0,r.terminal_time]); xlabel('t / s'); ylabel('Position / m');
title('Rear-axle centre'); legend('x','y','Location','best'); set(gca,'FontSize',11);
sgtitle(sprintf('Chapter 5 | Optimal states and controls | T = %.3f s',r.terminal_time));
drawnow;
end
