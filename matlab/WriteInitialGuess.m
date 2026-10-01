function WriteInitialGuess(r)
% Preserve the whole previous solution, including independent disc variables.
names = {'x','y','theta','v','a','phy','w','xf','yf','xr','yr'};
fid = fopen(fullfile('AmplInputs','InitialGuess.INIVAL'),'w');
if fid < 0, error('LIOM:WriteInput','Cannot write initial guess.'); end
for jj = 1 : numel(names)
    for ii = 1 : numel(r.x)
        fprintf(fid,'let %s[%d] := %.17g;\n',names{jj},ii,r.(names{jj})(ii));
    end
end
fprintf(fid,'let tf := %.17g;\n',r.terminal_time);
fclose(fid);
end
