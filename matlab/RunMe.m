%% 第五章配套源代码 / Chapter 5 companion code
% 中文图书：《非结构化场景自动驾驶轨迹规划技术》
% English translation: Trajectory Planning Techniques for Autonomous Driving
% in Unstructured Environments. This is a Chinese-language book.
% 上述英文名称仅为中文书名的译名，并非英文版图书。
% 本代码配套第五章《基于轻量化迭代优化的自主泊车轨迹规划鲁棒性增强方法》。
% 请结合式 (5-1)--(5-4)、算法 5-1 和表 5-1 阅读本实现。
%
% 使用本代码开展研究，请在论文、技术报告中引用 / Please cite:
% B. Li et al., "Optimization-based Trajectory Planning for Autonomous Parking
% with Irregularly Placed Obstacles: A Lightweight Iterative Framework,"
% IEEE Transactions on Intelligent Transportation Systems, 23(8), 11970-11981, 2022.
% https://ieeexplore.ieee.org/document/9344631 (first published online in 2021).
%
% Windows 64-bit / MATLAB R2021b / Navigation Toolbox / Image Processing Toolbox.
% 直接运行：200 个配置点、表 5-1 参数、原始 CaseNo_12.mat。
% AMPL/IPOPT(MA27) 通过 .run 与 TXT 交互，无须 MATLAB API。
% 最终仅生成两张静态图：轨迹迭代/最优足迹；最优状态/控制量。
% 初始化搜索以 P-code 提供；可读的 LIOM 方法与模型见同目录代码。
% README 提供完整函数说明、架构、安装方式和引用。
% Original copyright (C) 2025 Bai Li; GNU General Public License v3.0.
result = RunParkingDemo();
