# Python / CasADi 版

本版独立实现第五章安全行驶走廊构造、二次罚项模型和算法 5-1 外层迭代。使用随附 HA* 初始参考数据；无需 MATLAB 或 AMPL。模型、200 点参数、结果图及论文引用见[首页](../README.md)。

## 安装

已验证 Python 3.13、CasADi 3.7.2、NumPy 2.5.3、Matplotlib 3.11.2，Windows 64 位。建议使用独立环境，在 `python/` 内执行：

```bash
python -m venv .venv
# Windows PowerShell:
.venv\Scripts\Activate.ps1
# Linux/macOS: source .venv/bin/activate
python -m pip install -r requirements.txt
```

[CasADi 官方安装文档](https://web.casadi.org/get/)说明 Python 包的安装方法。所装 CasADi 需要包含 IPOPT 插件。

## 运行

**与书中相同的 MA27**：安装带 MA27 的 HSL 动态库，然后明确指定其路径，或设置 `LIOM_HSL_LIBRARY` 环境变量：

```bash
python runme.py --hsl-library /absolute/path/to/libcoinhsl.so
```

Windows 路径示例：

```powershell
python runme.py --hsl-library "C:\solvers\libcoinhsl.dll"
```

HSL 获取与构建见 [IPOPT 官方 HSL 安装说明](https://coin-or.github.io/Ipopt/INSTALL.html#DOWNLOAD_HSL)。通过 `ipopt.hsllib` 传入动态库，不需要重新编译优化模型。动态库及其 Fortran/BLAS 等依赖必须与 Python/CasADi 的操作系统、位数匹配。Windows 中程序会把所指定 HSL 库的目录加入运行库搜索路径；Linux 中将相关依赖目录加入 `LD_LIBRARY_PATH`。若使用已将 MA27 链接进 IPOPT 的 CasADi，则直接执行 `python runme.py`。

**使用 CasADi 包中的 MUMPS**：无需另装 HSL，明确选择后端：

```bash
python runme.py --linear-solver mumps
```

此命令保持第五章的模型和 LIOM 参数，但线性求解器不同于书中的 MA27。默认值仍为 `ma27`，不会自动降级为 MUMPS。

成功后弹出两张图，同时在 `results/` 保存 `trajectory.png`、`profiles.png` 和以下 CSV：

| 文件 | 内容 |
| --- | --- |
| `trajectory.csv` | 最优 `x,y,theta,v,a,phy,w,xf,yf,xr,yr`，一行一个配置点 |
| `summary.csv` | 总时间、代价、不可行度、解变化量、迭代数 |
| `iterations.csv` | 每轮罚权、名义代价及两个收敛指标 |
| `intermediate.csv` | 各轮轨迹，供第一张图展示迭代过程 |

不弹出窗口但保存两张图：在命令末尾加 `--no-show`。可用 `--data PATH` 和 `--output PATH` 指定输入、输出目录。

## 文件

- `runme.py`：命令行、加载输入、求解与绘图。
- `liom.py`：`Problem` 读取数据；`Corridors` 重建走廊；`bounds` 设置硬约束；`build_solver` 构造 CasADi 模型；`solve` 执行算法 5-1；`validate` 核验离散结果；`save_results` 保存结果。
- `plot_results.py`：静态轨迹/足迹和状态/控制量两张图，可单独用于已有结果。
- `data/`：MATLAB 保护模块导出的参考轨迹、场景和地图，格式见 [data/README.md](data/README.md)。

程序仅在 IPOPT 报告成功且两个外层停止条件同时满足时输出收敛结果。若 HSL 未安装或运行库不兼容，会报告求解失败；检查 `--hsl-library`，或明确选择 MUMPS。
