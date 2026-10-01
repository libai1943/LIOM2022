# MATLAB 版

配套中文图书《非结构化场景自动驾驶轨迹规划技术》第五章。书籍说明、技术对应、示例图及论文引用见[仓库首页](../README.md)。

## 运行

已验证环境：Windows 64 位、MATLAB R2021b，安装 **Navigation Toolbox**（初始化搜索使用 Reeds–Shepp 连接）和 **Image Processing Toolbox**（膨胀占据地图）。附带的 P-code 由 R2021b 生成。

1. 按下节完成首次求解器安装，保留本目录中的 AMPL/IPOPT 可执行文件和对应 DLL，以及 `ParkingBenchmarks/`。已有原始运行文件时可直接沿用。
2. 在 MATLAB 中打开 `RunMe.m`，点击运行；或进入本目录后输入 `RunMe`。
3. 等待 HA* 初始化和 LIOM 三轮迭代，最终显示两张静态图，工作区返回 `result`。

无需配置 MATLAB–AMPL API。`SolveIntermediate.run` 是 AMPL 指令文件，由 `AMPL.exe` 执行；每轮以 `AmplInputs/` 写入数据，IPOPT 计算后将标志位和变量写入 `AmplResults/`，MATLAB 检查并读回。两个交换目录必须保留，运行中也会自动补建。

GitHub 版不附带第三方 EXE/DLL，首次安装步骤如下：

1. 从 [AMPL 官方页面](https://ampl.com/try-ampl/request-a-full-trial/)取得 Windows 64 位 AMPL 并配置有效许可，将 `AMPL.exe` 放入本目录。
2. 按 [IPOPT 官方安装说明](https://coin-or.github.io/Ipopt/INSTALL.html)安装具有 AMPL 接口的 `ipopt.exe`，及其配套 BLAS、LAPACK、Fortran 等 DLL。将可执行文件和配套运行库放入本目录。
3. 安装 [HSL/MA27](https://coin-or.github.io/Ipopt/INSTALL.html#DOWNLOAD_HSL)，使用已链接 MA27 的 IPOPT，或按该 IPOPT 发行包的动态加载方式配置 HSL。保持 `ipopt.opt` 中 `linear_solver ma27`，与书中一致。完整原始运行文件的用户可直接保留原文件组合。
4. 在 Windows 终端中进入本目录，分别执行 `.\AMPL.exe -v` 和 `.\ipopt.exe -v`，确认两者能启动，再在 MATLAB 中运行 `RunMe`。

缺少运行文件时入口会给出明确错误。若提示许可证问题，请配置自己的有效 AMPL 许可；若 MA27 无法加载，请检查 HSL 与 IPOPT 及其依赖版本。求解状态与日志见 `AmplResults/solve_result_num.txt` 和 `solver.log`。运行失败会报错，不读取旧的最优解充当新结果。

## 主要函数

| 文件 | 功能 |
| --- | --- |
| `RunMe.m` | 中英文书籍介绍、论文引用及一键运行入口 |
| `RunParkingDemo.m` | 管理工作目录、固定初始化随机种子、加载场景、调用搜索/优化/绘图 |
| `LoadCase.m` | 加载原始 `ParkingBenchmarks/CaseNo_12.mat` |
| `InitializeParams.m` | 车辆、200 点离散、搜索、罚权及走廊参数；生成原始及膨胀地图 |
| `CompleteInitialGuess.m` | 形成含状态、控制、圆心及总时间的完整初值 |
| `OptimizeTrajectoryViaLIOM.m` | 算法 5-1：逐轮重建走廊、求解、更新参考和罚权、检查两个停止条件 |
| `ConstructSafeTravelCorridors.m` | 调整圆心种子并交替扩张轴对齐矩形，写出前/后圆走廊 |
| `SolutionVector.m` | 将完整解按固定顺序展开，用于计算式 (5-4) 中的解变化量 |
| `WriteBasicParameterFile.m` | 写车辆参数、离散参数、代价系数及当前罚权 |
| `WriteBoundaryValues.m` | 写起终点位姿与车辆边界条件 |
| `WriteInitialGuess.m` | 将上一轮完整解以 17 位精度写为下一轮初值 |
| `SolveIntermediateProblem.m` | 清理本轮结果文件、运行 AMPL 指令、保存求解日志、读取结果 |
| `LoadAmplSolution.m` | 检查求解状态、配置点数量与数值有效性，读回 11 组变量及总时间 |
| `ValidateSolution.m` | 检查离散 Euler/几何残差、边界/走廊/变量范围及双圆避障 |
| `DrawResults.m` | 只生成轨迹足迹图、最优状态控制图两张静态图 |
| `DrawParkingScenario.m` | 绘制障碍物及场景坐标系 |
| `DrawTrajFootprints.m` | 绘制稀疏展示的最优车辆足迹；先于轨迹绘制 |
| `CreateVehiclePolygon.m` | 由后轴中心位姿计算车身四角，用于静态足迹 |
| `RegulateAngle.m` | 将角度规整到初始化搜索所用范围 |
| `ExportReferenceData.m` | 导出算法输入到 C++/Python 的 `data/`，不导出优化答案 |
| `SearchTrajectoryViaFTHA.p` | 受保护的 HA* 初始化搜索入口 |
| `SearchAStarPath.p` | 受保护的二维 A* 辅助搜索 |
| `ResamplePath.p` | 受保护的搜索结果重采样与初始时间参数化 |

模型文件 `NLP.mod` 对应第五章的软化中间问题；`SolveIntermediate.run` 负责读写模型和变量；`ipopt.opt` 指定 IPOPT–MA27 及数值求解选项。

## 跨语言输入导出

运行 `RunMe` 后，可重新导出当前 HA* 参考：

```matlab
ExportReferenceData('../python/data');
ExportReferenceData('../cpp/data');
```

两份数据与场景成套使用，字段见各自 `data/README.md`。本次发布已有导出数据，因此运行 C++/Python 时无需先运行 MATLAB。初始化的三个函数仅以 P-code 发布，可视化与 LIOM 主体以 M 文件提供。
