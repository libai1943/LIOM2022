function handle = DrawTrajFootprints(x,y,theta)
% Plot representative footprints before overlaying the two trajectory lines.
s = [0,cumsum(hypot(diff(x),diff(y)))];
[distance,index] = unique(s,'stable');
samples = unique(round(interp1(distance,index,linspace(0,s(end),65))));
for ii = samples
    V = CreateVehiclePolygon(x(ii),y(ii),theta(ii),2);
    V = [V.x(:),V.y(:)];
    handle = plot(V(:,1),V(:,2),'Color',[0.40,0.71,0.91], ...
        'LineWidth',0.65,'HandleVisibility','off');
end
set(handle,'HandleVisibility','on','DisplayName','Optimal footprints');
end
