function report = ValidateSolution(r)
% Only the discrete configuration points are checked.
global params_
g = params_.vehicle; b = params_.task; dt = r.terminal_time/(numel(r.x)-1); ix = 1:numel(r.x)-1;
e = [diff(r.x)-dt*r.v(ix).*cos(r.theta(ix)),diff(r.y)-dt*r.v(ix).*sin(r.theta(ix)), ...
    diff(r.v)-dt*r.a(ix),diff(r.theta)-dt*r.v(ix).*tan(r.phy(ix))/g.lw,diff(r.phy)-dt*r.w(ix)];
d = [r.xf-r.x-g.f2p*cos(r.theta);r.yf-r.y-g.f2p*sin(r.theta); ...
    r.xr-r.x-g.r2p*cos(r.theta);r.yr-r.y-g.r2p*sin(r.theta)];
report.infeasibility = sum(e.^2)+sum(sum(d(:,2:end).^2));
report.max_euler_residual = max(abs(e)); report.max_geometry_residual = max(abs(d(:)));
boundary = [r.x(1)-b.x0,r.y(1)-b.y0,r.theta(1)-b.theta0,r.x(end)-b.xf,r.y(end)-b.yf,r.theta(end)-b.thetaf, ...
    r.v([1,end]),r.phy([1,end]),r.a(end),r.w(end)];
report.max_boundary_residual = max(abs(boundary));
cr = r.rear_corridor; cf = r.front_corridor;
report.max_corridor_violation = max([0,cr(:,1)'-r.xr,r.xr-cr(:,2)',cr(:,3)'-r.yr,r.yr-cr(:,4)', ...
    cf(:,1)'-r.xf,r.xf-cf(:,2)',cf(:,3)'-r.yf,r.yf-cf(:,4)']);
report.max_bound_violation = max([0,abs(r.v)-g.vmax,abs(r.a)-g.amax,abs(r.phy)-g.phymax,abs(r.w)-g.wmax]);
centres = [r.x+g.r2p*cos(r.theta),r.x+g.f2p*cos(r.theta);r.y+g.r2p*sin(r.theta),r.y+g.f2p*sin(r.theta)]';
clearance = Inf;
for ii = 1 : params_.obstacle.num_obs
    obs = params_.obstacle.obs{ii}; vertices = [obs.x(:),obs.y(:)];
    if any(inpolygon(centres(:,1),centres(:,2),vertices(:,1),vertices(:,2))), error('LIOM:Collision','A disc centre lies in an obstacle.'); end
    vertices = [vertices;vertices(1,:)]; dist = Inf(size(centres,1),1);
    for jj = 1:size(vertices,1)-1
        edge = vertices(jj+1,:)-vertices(jj,:);
        if sum(edge.^2)==0, continue; end
        u = max(0,min(1,(centres-vertices(jj,:))*edge'/sum(edge.^2)));
        delta = centres-vertices(jj,:)-u.*edge;
        dist = min(dist,hypot(delta(:,1),delta(:,2)));
    end
    clearance = min(clearance,min(dist)-g.dual_disk_radius);
end
report.min_disc_clearance = clearance;
assert(report.infeasibility < params_.opti.feasibility_tolerance && r.difference < params_.opti.diff_tolerance,'Both convergence criteria must hold.');
assert(max([report.max_boundary_residual,report.max_corridor_violation,report.max_bound_violation,-clearance]) < 1e-5,'Discrete hard constraints failed.');
assert(abs(report.infeasibility-r.infeasibility)<1e-8,'Penalty readback mismatch.');
end
