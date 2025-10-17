function PlotOptimalProfiles()
global params_
% 1) 读取 AMPL 求解结果
[params_.liom.x, params_.liom.y, params_.liom.theta, ...
    params_.liom.v, params_.liom.a, params_.liom.phy, ...
    params_.liom.w, params_.liom.terminal_time] = LoadAmplSolution();


% 3) 构造均匀时间轴（与NLP离散一致）
N  = numel(params_.liom.v);
Tf = params_.liom.terminal_time;
if Tf <= 0, Tf = 1; end
t  = linspace(0, Tf, N);

% 4) 画 2×3 子图：v, a, phy, w, theta（theta在右下角、标红）
figure('Color','w');
tiledlayout(2,3,'TileSpacing','compact','Padding','compact');

% v(t)
nexttile(1);
plot(t, params_.liom.v, 'Color', [0 0 0], 'LineWidth', 2.3);
grid on; xlabel('t / s'); ylabel('v / (m s^{-1})');
title('速度变量');
axis tight; grid minor

% a(t)
nexttile(4);
plot(t, params_.liom.a, 'Color', [1.00, 0 0], 'LineWidth', 2.3);
grid on; xlabel('t / s'); ylabel('a / (m s^{-2})');
title('加速度变量');
axis tight; grid minor

% \phi(t)
nexttile(2);
plot(t, params_.liom.phy, 'Color', [0 0 0], 'LineWidth', 2.3);
grid on; xlabel('t / s'); ylabel('\phi / rad');
title('前轮转动角度变量');
axis tight; grid minor

% w(t)
nexttile(5);
plot(t, params_.liom.w, 'Color', [1.00, 0 0], 'LineWidth', 2.3);
grid on; xlabel('t / s'); ylabel('\omega / (rad s^{-1})');
title('前轮转动角速度变量');
axis tight; grid minor

% 空出 middle-right（可按需添加其他量）
nexttile(6);
axis off;

% \theta(t) —— 右下角，标红
nexttile(3);
plot(t, params_.liom.theta, 'Color', [0 0 0], 'LineWidth', 2.3);
grid on; xlabel('t / s'); ylabel('\theta / rad');
title('车头朝向角度变量');
axis tight; grid minor
end
