# MspaceViewer

一个基于 C++ Qt 与 OpenSceneGraph 的点云可视化与配准应用程序，用于加载、显示和配准 TLS（地面激光扫描）与 ULS（无人机激光扫描）的 LiDAR 点云数据。

## 项目简介

MspaceViewer 是一款面向激光扫描（LiDAR）点云数据的桌面应用程序，支持大规模点云的加载、分层八叉树组织、三维可视化以及多源点云配准。

### 核心功能

- **点云加载与可视化**：支持 LAS/LAZ 格式点云文件，通过分层八叉树结构实现高效的 LOD（多细节层次）渲染
- **TLS-TLS 配准**：基于菱形描述子与 FPFH 特征匹配的地面激光扫描点云配准
- **ULS-TLS 配准**：基于 PCA 描述子与极坐标离散化的无人机到地面点云粗配准
- **D4PCR 离群值鲁棒配准**：使用区间 stabbing 算法实现高离群值比例下的 4DOF 配准
- **Forest TLS Registration**：基于 small_gicp + GTSAM 的森林场景点云配准
- **多语言支持**：内置中文/英文界面切换

## 项目架构

### 系统架构

系统采用模块化 C++ 架构，基于 Qt 构建 GUI，通过 OpenSceneGraph 实现三维渲染，PCL 提供点云算法支持。

```text
┌─────────────────────────────────────────────────────────┐
│                      MainWindow (Qt GUI)                 │
│  ┌──────────────┐  ┌───────────────┐  ┌──────────────┐ │
│  │ 数据管理树视图 │  │ osgQOpenGLWidget│  │ 配准对话框   │ │
│  │ (QStandardItem)│  │ (OSG 渲染视图) │  │ TLS/ULS/D4PCR│ │
│  └──────┬───────┘  └───────┬───────┘  └──────┬───────┘ │
│         │                  │                  │          │
└─────────┼──────────────────┼──────────────────┼──────────┘
          │                  │                  │
          ▼                  ▼                  ▼
┌─────────────────┐ ┌───────────────┐ ┌────────────────────┐
│  MspaceOctree   │ │   osgQt       │ │ 配准算法模块        │
│  八叉树数据结构  │ │ Qt-OSG 集成层  │ │ TLSRegistration    │
│  LAS/LAZ→Octree │ │ 渲染线程管理   │ │ ULS-TLS            │
│  Poisson/Random │ └───────────────┘ │ D4PCR              │
│  采样           │                   │ Forest_TLS_Reg     │
└─────────────────┘                   └────────────────────┘
```

### 目录结构

```text
MspaceViewer/
├── CMakeLists.txt              # 根 CMake 构建配置（C++20）
├── vcpkg.json                  # vcpkg 依赖清单
├── MainWindow/                 # 主 Qt GUI 应用
│   ├── src/
│   │   ├── main.cpp
│   │   ├── MainWindow.cpp
│   │   ├── RegistrationForm_TLS.cpp
│   │   ├── RegistrationForm_ULS.cpp
│   │   ├── OutlierRemovalRegistration.cpp
│   │   └── ForestRegistration.cpp
│   ├── include/
│   └── resource/               # UI 文件、资源文件
├── MspaceOctree/               # 基于八叉树的点云数据结构
│   ├── libs/laszip/            # LASzip 压缩库
│   └── libs/json/              # JSON 解析库
├── osgQt/                      # Qt 与 OpenSceneGraph 集成层
├── osgdb_bin/                  # OSG 数据库插件（二进制点云加载）
├── TLSRegistration/            # TLS-TLS 配准算法
├── ULS-TLS/                    # ULS-TLS 配准算法
├── D4PCR/                      # D4PCR 离群值鲁棒配准
└── Forest_TLS_Reg/             # 森林场景配准（small_gicp + GTSAM）
    └── thirdParty/
        ├── small_gicp/         # small_gicp 库
        └── CSF-master/         # Cloth Simulation Filter
```

### 核心模块职责

| 模块 | 职责 |
|------|------|
| **MainWindow** | 主窗口与 UI 交互，数据管理树视图、配准对话框、多语言切换 |
| **MspaceOctree** | LAS/LAZ 文件转换为分层八叉树格式，支持 `MSpaceOCTree` 构建与 `MSpaceNode` LOD 渲染节点 |
| **osgQt** | Qt 与 OpenSceneGraph 的桥接层，实现 QOpenGLWidget 内嵌 OSG 渲染 |
| **osgdb_bin** | OSG 数据库插件，用于读取二进制格式的点云数据 |
| **TLSRegistration** | TLS-TLS 配准算法：菱形描述子提取、FPFH 特征匹配 |
| **ULS-TLS** | `RegistrationU2T` 类：GridMinimum 滤波、PCA 描述子筛选、Z 值极坐标离散化估计旋转角 |
| **D4PCR** | `Decoupling` 类：区间 stabbing 算法，针对高离群值比例的鲁棒 4DOF 配准 |
| **Forest_TLS_Reg** | 基于 small_gicp + GTSAM 的森林场景点云配准，含 CSF 地面滤波 |

## 环境依赖与安装

### 系统要求

| 项目 | 要求 |
|------|------|
| 操作系统 | Windows 10/11 (64-bit) |
| 编译器 | MSVC v143+ (Visual Studio 2022) |
| CMake | >= 3.21 |
| C++ 标准 | C++20 |

### 依赖项概览

| 依赖 | 版本 | 安装方式 | 用途 |
|------|------|----------|------|
| **Qt** | 6.11.1 | [Qt 在线安装器](https://www.qt.io/download-qt-installer) | GUI 框架（Widgets, Concurrent, Core, Gui, OpenGL, OpenGLWidgets） |
| **PCL** | 1.14.1 | [All-in-One Installer](https://github.com/PointCloudLibrary/pcl/releases) | 点云处理（common, io, filters, features, registration, search, kdtree, segmentation, visualization） |
| **OpenCV** | 4.7 | [Releases](https://github.com/opencv/opencv/releases) | 图像处理（core, imgproc） |
| **OpenNI2** | 2.2.0 | [下载](https://structure.io/openni) | 深度传感器接口 |
| **OpenSceneGraph** | 3.6.5 | vcpkg 自动安装 | 三维渲染引擎 |
| **libLAS** | 1.8.1 | vcpkg 自动安装 | LAS 文件读写 |
| **Boost** | 1.91.0 | vcpkg 自动安装 | filesystem, graph, ptr-container, regex, serialization 等 |
| **Eigen3** | - | PCL 自带 | 线性代数运算 |
| **OpenMP** | - | MSVC 内置 | 并行计算 |
| **small_gicp** | - | 嵌入式（thirdParty） | 点云配准 |
| **nlohmann-json** | 3.12.0 | vcpkg 自动安装 | JSON 解析 |
| **FreeType** | 2.14.3 | vcpkg 自动安装 | 字体渲染 |

### 第一步：安装 Visual Studio 2022

1. 下载 [Visual Studio 2022](https://visualstudio.microsoft.com/)
2. 安装时选择 **"使用 C++ 的桌面开发"** 工作负载
3. 确保包含 **MSVC v143** 编译器和 **C++ CMake tools for Windows**

### 第二步：安装 CMake

1. 下载 [CMake](https://cmake.org/download/) >= 3.21
2. 安装时勾选 **"Add CMake to the system PATH"**

### 第三步：安装 Qt 6

1. 下载并安装 [Qt 在线安装器](https://www.qt.io/download-qt-installer)
2. 登录 Qt 账号，选择安装 **Qt 6.11.1**（或更新版本）
3. 组件选择：勾选 **MSVC 2022 64-bit** 编译器套件
4. 记下安装路径，默认为 `D:/Qt/6.11.1/msvc2022_64`

### 第四步：安装 PCL 1.14.1

1. 下载 [PCL 1.14.1 All-in-One](https://github.com/PointCloudLibrary/pcl/releases/tag/pcl-1.14.1)
2. 运行安装程序，选择 **"Add PCL to the system PATH for all users"**
3. 默认安装路径：`D:/Program Files/PCL 1.14.1`
4. 安装包含：PCL 库、Eigen3、Boost、FLANN、VTK、OpenNI2

### 第五步：安装 OpenCV 4.7

1. 下载 [OpenCV 4.7 Windows Pack](https://github.com/opencv/opencv/releases/tag/4.7.0)
2. 解压到 `D:/Software/opencv`
3. 注意：OpenCV Windows Pack 仅包含 **vc16 (VS2019)** 预编译库，但与 VS2022 **兼容**

### 第六步：安装 vcpkg

```bash
# 克隆 vcpkg
cd D:/
git clone https://github.com/microsoft/vcpkg.git
cd vcpkg

# 引导安装 vcpkg
.\bootstrap-vcpkg.bat
```

### 第七步：安装 vcpkg 依赖

```bash
cd D:/MspaceViewer

# 方式一：使用 vcpkg manifest 模式（推荐，会在 cmake configure 时自动安装）
# 无需手动操作，运行 cmake 时会自动根据 vcpkg.json 安装依赖

# 方式二：手动安装
vcpkg install --triplet x64-windows-release
```

vcpkg 会自动安装以下依赖（见 `vcpkg.json`）：
- `osg[core,freetype]` — OpenSceneGraph
- `liblas` — LAS 文件读写
- `boost-filesystem` — 文件系统操作
- `boost-graph` — 图算法
- `boost-ptr-container` — 指针容器

## 构建

### 使用 CMake 命令行（推荐）

```bash
cd E:/Code/MspaceViewer

# 配置 CMake（使用 vcpkg toolchain）
cmake -B build -G "Visual Studio 17 2022" -A x64 `
  -DCMAKE_TOOLCHAIN_FILE=D:/vcpkg/scripts/buildsystems/vcpkg.cmake `
  -DVCPKG_TARGET_TRIPLET=x64-windows-release

# 编译 Release 版本
cmake --build build --config Release
```

**常用构建命令：**

```bash
# 仅编译主程序
cmake --build build --config Release --target MspaceViewer

# 编译全部模块
cmake --build build --config Release

# 编译 Debug 版本
cmake --build build --config Debug
```

### 使用 Visual Studio

1. 打开 Visual Studio 2022
2. 选择 **"打开本地文件夹"**，选择项目根目录
3. Visual Studio 会自动检测 `CMakeLists.txt` 并配置项目
4. 在工具栏选择 **Release** 配置
5. 选择 **MspaceViewer.exe** 为启动目标
6. 按 **Ctrl+F5** 编译运行

## 运行

### 复制运行时 DLL

编译完成后，需要将以下 DLL 复制到 `build/bin/Release/` 目录（或添加到系统 PATH）：

```powershell
# PCL DLLs（Release 版本，不带 'd' 后缀）
Get-ChildItem "D:/Program Files/PCL 1.14.1/bin/pcl_*.dll" |
  Where-Object { $_.Name -notmatch 'd\.dll$' } |
  Copy-Item -Destination "build/bin/Release/"

# OpenCV DLL
Copy-Item "D:/Software/opencv/build/x64/vc16/bin/opencv_world470.dll" "build/bin/Release/"

# VTK DLLs
Get-ChildItem "D:/Program Files/PCL 1.14.1/3rdParty/VTK/bin/*.dll" |
  Where-Object { $_.Name -notmatch 'd\.dll$' } |
  Copy-Item -Destination "build/bin/Release/"

# FLANN DLL
Copy-Item "D:/Program Files/PCL 1.14.1/3rdParty/FLANN/lib/flann_cpp_s.dll" "build/bin/Release/"

# OpenNI2 DLL
Copy-Item "C:/Program Files/OpenNI2/Redist/OpenNI2.dll" "build/bin/Release/"

# vcpkg DLLs（osg, liblas, Boost 等）
Get-ChildItem "build/vcpkg_installed/x64-windows-release/bin/*.dll" |
  Copy-Item -Destination "build/bin/Release/"
```

### 启动程序

```bash
# 方式一：直接运行
build/bin/Release/MspaceViewer.exe

# 方式二：从项目根目录运行
.\build\bin\Release\MspaceViewer.exe
```

**注意**：首次启动时可能会看到 DPI 感知相关的警告信息（`SetProcessDpiAwarenessContext() failed`），这是正常的，不影响程序使用。

## 使用说明

### 加载点云

1. 启动应用程序，主窗口默认以最大化显示
2. 通过菜单或工具栏加载 LAS/LAZ 格式的点云文件
3. 点云将自动转换为八叉树格式并在 OSG 视图中渲染

### 点云配准

#### TLS-TLS 配准

1. 打开 TLS 配准对话框（`RegistrationForm_TLS`）
2. 选择源点云与目标点云
3. 执行配准（基于菱形描述子与 FPFH 特征匹配）

#### ULS-TLS 配准

1. 打开 ULS 配准对话框（`RegistrationForm_ULS`）
2. 加载无人机点云（ULS）与地面点云（TLS）
3. 执行配准（基于 PCA 描述子与极坐标离散化）

#### D4PCR 离群值鲁棒配准

1. 打开离群值剔除配准对话框（`OutlierRemovalRegistration`）
2. 选择参与配准的点云
3. 执行配准（区间 stabbing 算法）

#### Forest TLS 配准

1. 打开森林配准对话框（`ForestRegistration`）
2. 加载森林场景点云
3. 执行配准（基于 small_gicp + GTSAM）

## 快速开始（一行命令）

已安装所有依赖后，执行以下命令即可完成构建：

```powershell
# 配置 + 编译
cmake -B build -G "Visual Studio 17 2022" -A x64 -DCMAKE_TOOLCHAIN_FILE=D:/vcpkg/scripts/buildsystems/vcpkg.cmake -DVCPKG_TARGET_TRIPLET=x64-windows-release 2>&1; cmake --build build --config Release 2>&1
```

## 贡献指南

### 代码规范

- C++20 标准
- 头文件使用 `#pragma once`
- 导出宏统一使用：`MSPACEOCTREE_LIB_EXPORT`、`OSGQT_EXPORT`、`DLL_EXPORT`
- 注释可使用中文或英文
- PCL 智能指针使用 `pcl::PointCloud<pcl::PointXYZ>::Ptr`
- OSG 引用指针使用 `osg::ref_ptr<osg::Node>`
- Qt signal/slot 用于 UI 回调

## 许可证

本项目依赖的第三方库遵循各自许可证：

- **Qt**：[LGPL/GPL 商业许可](https://www.qt.io/licensing/)
- **OpenSceneGraph**：[LGPL/OSGPL](https://openscenegraph.com/)
- **PCL**：[BSD](https://github.com/PointCloudLibrary/pcl/blob/master/LICENSE.txt)
- **Eigen3**：[MPL2](https://eigen.tuxfamily.org/)
- **OpenCV**：[Apache 2.0](https://opencv.org/license/)
- **LASzip**：[LGPL](https://github.com/LASzip/LASzip)
- **nlohmann/json**：[MIT](https://github.com/nlohmann/json)
- **small_gicp**：[BSD](https://github.com/koide3/small_gicp)
- **CSF**：[GPL](https://github.com/jianboqian/CSF)

本项目自身的开源协议请参见仓库中的 `LICENSE` 文件。
