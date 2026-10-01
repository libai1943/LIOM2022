# C++ / CasADi 版

走廊生成、变量边界、CasADi/IPOPT 优化、算法 5-1 迭代及离散结果检查均在 C++ 中完成。Python 入口只启动编译后的程序并绘制两张静态图。无需 MATLAB、AMPL 或 ROS。书中技术对应和引用见[首页](../README.md)。

## 依赖

- C++17 编译器、CMake 3.16 或更新版本。
- CasADi 3.7.2 C++ 库、头文件、CMake 配置，以及 IPOPT 插件。
- 与书中相同的 MA27：另行安装兼容的 HSL 动态库；也可明确选择 MUMPS。
- 绘图：Python、NumPy、Matplotlib，执行 `python -m pip install -r requirements.txt`。

CasADi 获取方式见[官方安装文档](https://web.casadi.org/get/)。使用与编译器 ABI 匹配的二进制包；Windows 的 MinGW 库与 MSVC 库不能混用。HSL 获取见 [IPOPT 官方说明](https://coin-or.github.io/Ipopt/INSTALL.html#DOWNLOAD_HSL)。可参考作者的 [CartesianPlanner](https://github.com/libai1943/CartesianPlanner) 了解 C++ 中的 CasADi 接入方式。

## Windows：已验证的构建方式

本次验证使用 Windows 64 位、w64devkit GCC 16.2.0、CMake 4.4，以及 `pip install casadi==3.7.2` 所含的 MinGW C++ 头文件、库和 IPOPT 插件。该 pip 包在此仅作为 C++ 依赖的分发包，优化仍由编译后的 `liom.exe` 执行。

1. 从 [w64devkit 官方发行页](https://github.com/skeeto/w64devkit/releases)下载并解压，将 `bin` 加入 PATH。
2. 安装 Python 依赖及 CasADi：

```powershell
python -m pip install casadi==3.7.2
python -m pip install -r requirements.txt
```

3. 在 `cpp/` 目录用 PowerShell 构建。下方 `C:\tools\w64devkit` 替换为实际目录：

```powershell
$compilerDir = 'C:\tools\w64devkit\bin'
$casadiDir = python -c "import casadi,pathlib; print(pathlib.Path(casadi.__file__).parent)"
$env:PATH = "$compilerDir;$casadiDir;" + $env:PATH
$env:CASADIPATH = $casadiDir
cmake -S . -B build -G "MinGW Makefiles" -DCMAKE_BUILD_TYPE=Release "-Dcasadi_DIR=$casadiDir/cmake" "-DCMAKE_CXX_COMPILER=$compilerDir/g++.exe" "-DCMAKE_MAKE_PROGRAM=$compilerDir/mingw32-make.exe"
cmake --build build --parallel
```

4. 安装 MA27/HSL 后运行（替换实际 DLL 路径）：

```powershell
python runme.py --hsl-library 'C:\solvers\libcoinhsl.dll'
```

或显式选择 CasADi 包中自带的 MUMPS，直接运行：

```powershell
python runme.py --linear-solver mumps
```

新开终端时需要重新设置 `PATH`、`CASADIPATH`。HSL 与依赖 DLL 位于同一目录时，程序会将该目录追加到搜索路径，优先保留所安装的 CasADi 运行库，避免混用不同版本的 IPOPT DLL。

## Linux：依赖与构建示例

以下为 Linux 安装指引，本次实际运行验证在 Windows 上完成。若已有带 IPOPT 的 CasADi C++ 安装，可跳过依赖构建，直接指定 `casadi_DIR`。

Ubuntu/Debian 可先安装编译工具及 IPOPT 开发包，再构建 CasADi（IPOPT 开发包包含其发行版提供的线性代数依赖）：

```bash
sudo apt-get update
sudo apt-get install build-essential cmake git pkg-config coinor-libipopt-dev python3-venv
git clone --branch 3.7.2 --depth 1 https://github.com/casadi/casadi.git casadi-src
cmake -S casadi-src -B casadi-build -DCMAKE_BUILD_TYPE=Release -DWITH_IPOPT=ON -DWITH_PYTHON=OFF -DCMAKE_INSTALL_PREFIX="$PWD/casadi-install"
cmake --build casadi-build --parallel
cmake --install casadi-build
```

进入本仓库 `cpp/`，指定上述实际安装位置（`casadi_DIR` 是包含 `casadi-config.cmake` 的目录）：

```bash
cmake -S . -B build -DCMAKE_BUILD_TYPE=Release -Dcasadi_DIR=/absolute/path/to/casadi-install/lib/cmake/casadi
cmake --build build --parallel
export LD_LIBRARY_PATH=/absolute/path/to/casadi-install/lib:$LD_LIBRARY_PATH
export CASADIPATH=/absolute/path/to/casadi-install/lib
python3 -m venv .venv
source .venv/bin/activate
python -m pip install -r requirements.txt
python runme.py --hsl-library /absolute/path/to/libcoinhsl.so
```

安装的 IPOPT 若不包含 MA27，按官方 HSL 指引安装动态库及其依赖；也可明确使用 `--linear-solver mumps`。书中采用 MA27，默认值与之保持一致，程序不会自动改用其他后端。

## 直接运行 C++ 可执行文件

```bash
./build/liom --data data --output results --linear-solver ma27 --hsl-library /absolute/path/to/libcoinhsl.so
python plot_results.py --data data --output results
```

Windows 对应 `build\liom.exe`。`--hsl-library` 可改用环境变量 `LIOM_HSL_LIBRARY`；MA27 若已链接进 IPOPT，可省略路径。完整入口 `runme.py` 可用 `--executable PATH` 指定其他构建目录，支持 `--no-show` 保存图片而不弹出窗口。

两张图片及变量 CSV 保存到 `results/`。直接运行 C++ 仅生成 CSV，Python 绘图步骤读取这些 CSV，不执行优化。文件字段与 [Python 版](../python/README.md)一致。

## 文件职责

| 文件 | 功能 |
| --- | --- |
| `include/liom.hpp` | 问题、走廊和求解结果的接口与数据结构 |
| `src/corridor.cpp` | 初值展开、原始走廊扩张、完整变量上下界 |
| `src/optimizer.cpp` | CasADi SX 罚项模型、IPOPT 求解、算法 5-1 与离散点核验 |
| `src/main.cpp` | 参数读取、CSV 输入输出、运行库路径处理、错误退出 |
| `CMakeLists.txt` | 查找并链接 `casadi::casadi`，生成 C++17 可执行程序 |
| `runme.py` | 调用已编译 C++ 程序，再调用静态绘图 |
| `plot_results.py` | 轨迹/足迹、状态/控制量两张图 |
| `data/` | 保护模块导出的初始数据，见 [格式说明](data/README.md) |
