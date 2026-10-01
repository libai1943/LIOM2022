function r = CompleteInitialGuess(r)
global params_
g = params_.vehicle;
for entry = {'v','a','phy','w'}
    name = entry{1}; limit = g.([name,'max']);
    r.(name) = max(-limit,min(limit,r.(name)));
end
r.xf = r.x + g.f2p*cos(r.theta); r.yf = r.y + g.f2p*sin(r.theta);
r.xr = r.x + g.r2p*cos(r.theta); r.yr = r.y + g.r2p*sin(r.theta);
end
