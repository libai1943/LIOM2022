function z = SolutionVector(r)
% The complete solution vector in Algorithm 5-1, including T and all discs.
names = {'x','y','theta','v','a','phy','w','xf','yf','xr','yr'};
z = [];
for ii = 1 : numel(names), z = [z; r.(names{ii})(:)]; end
z = [z;r.terminal_time];
end
