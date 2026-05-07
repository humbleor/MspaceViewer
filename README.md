# MspaceViewer

一个基于 C++ Qt 与 OpenSceneGraph 的点云可视化与配准应用程序，用于加载、显示和配准 TLS（地面激光扫描）与 ULS（无人机激光扫描）的 LiDAR 点云数据。

## 项目简介

MspaceViewer 是一款面向激光扫描（LiDAR）点云数据的桌面应用程序，支持大规模点云的加载、分层八叉树组织、三维可视化以及多源点云配准。

### 核心功能

- **点云加载与可视化**：支持 LAS/LAZ 格式点云文件，通过分层八叉树结构实现高效的 LOD（多细节层次）渲染
- **TLS-TLS 配准**：基于菱形描述子与 FPFH 特征匹配的地面激光扫描点云配准
- **ULS-TLS 配准**：基于 PCA 描述子与极坐标离散化的无人机到地面点云粗配准
- **D4PCR 离群值鲁棒配准**：使用区间 stabbing 算法实现高离群值比例下的 4DOF 配准
- **多语言支持**：内置中文/英文界面切换


## 项目架构

### 系统架构

系统采用模块化 C++ 架构，基于 Qt 构建 GUI，通过 OpenSceneGraph 实现三维渲染，PCL 提供点云算法支持。

```
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
│  采样           │                   └────────────────────┘
└─────────────────┘
```

### 目录结构

```
MspaceViewer/
├── CMakeLists.txt            # CMake 构建配置（C++20）
├── cmake/                    # CMake 模块与依赖查找脚本
├── MainWindow/               # 主 Qt GUI 应用
│   ├── src/
│   │   └── main.cpp          # 程序入口
│   └── include/
├── MspaceOctree/             # 基于八叉树的点云数据结构
│   ├── libs/
│   │   ├── laszip/           # LASzip 压缩库
│   │   └── json/             # JSON 解析库
│   └── ...
├── osgQt/                    # Qt 与 OpenSceneGraph 集成层
│   ├── osgQOpenGLWidget      # QOpenGLWidget 子类，承载 OSG 视图器
│   ├── OSGRenderer           # 内部渲染线程管理
│   └── 自定义扩展            # StateEx, RenderStageEx, CullVisitorEx 等
├── osgdb_bin/                # OSG 数据库插件，加载二进制点云文件
├── TLSRegistration/          # TLS-TLS 点云配准（菱形描述子、FPFH 特征）
├── ULS-TLS/                  # UAV 到 TLS 点云粗配准（PCA 描述子、极坐标离散化）
├── D4PCR/                    # 4D 点云配准（区间 stabbing 离群值剔除）
└── build-fresh/              # 构建输出目录
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

### 数据流

1. LAS/LAZ 文件通过 `LasLoader` 加载
2. 经 `MSpaceOCTree::filesConverter()` 转换为八叉树格式
3. 以分层 `.bin` 文件 + `metadata.json` 存储
4. 可视化时通过 `potree_to_osg::convert()` 构建 `MSpaceNode` LOD 结构
5. 配准算法对滤波/采样后的点云执行对应计算

## 依赖与环境要求

### 开发环境

| 依赖 | 版本要求 | 用途 |
|------|----------|------|
| CMake | >= 3.16 | 构建系统 |
| C++ 编译器 | 支持 C++20 | MSVC (Visual Studio 2019/2022) |
| Qt | 5.15+ | GUI 框架（Widgets、Core、Gui、OpenGL、Concurrent） |
| OpenSceneGraph | 3.6+ | 三维渲染引擎（osgViewer、osgDB、osgUtil、osgGA） |
| PCL | 1.12+ | 点云处理库（配准算法模块） |
| Eigen3 | 3.3+ | 线性代数运算 |
| Boost | 1.74+ | 通用 C++ 库 |
| LASzip | 3.x | LAS/LAZ 文件压缩/解压缩 |
| nanoflann | - | KD-tree 实现 |
| OpenMP | - | 并行处理 |

### 操作系统

- Windows 10/11（主开发平台）

## 安装与构建

### 1. 克隆仓库

```bash
git clone <repository-url>
cd MspaceViewer
```

### 2. 安装依赖（vcpkg方式）

```bash
vcpkg install pcl osg gdal qt5-base --triplet=x64-windows
```

### 3. 配置并构建主项目

#### 使用 CMake（推荐）

```bash
cd MspaceViewer
mkdir build && cd build
cmake .. -G "Visual Studio 17 2022"
cmake --build . --config Release
```

#### 使用 Visual Studio

1. 在 Visual Studio 中打开 `MspaceViewer.sln`（或各子项目的 `.vcxproj` 文件）
2. 选择 **Release** 配置
3. 确保 Qt Visual Studio Tools 插件已安装并正确配置 Qt 路径
4. 生成解决方案

### 4. 运行

构建完成后，在输出目录中执行：

```
build\bin\Release\MspaceViewer.exe
```

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

## 贡献指南

### 开发流程

1. Fork 本仓库并创建功能分支
2. 遵循项目代码约定：
   - 头文件使用 `#pragma once`
   - PCL 智能指针使用 `pcl::PointCloud<pcl::PointXYZ>::Ptr`
   - OSG 引用指针使用 `osg::ref_ptr<osg::Node>`
   - Qt signal/slot 用于 UI 回调
3. 提交 Pull Request，描述变更内容与测试方法

### 代码规范

- C++20 标准
- 头文件使用 `#pragma once`
- 导出宏统一使用：`MSPACEOCTREE_LIB_EXPORT`、`OSGQT_EXPORT`、`DLL_EXPORT`
- 注释可使用中文或英文

## 许可证

本项目依赖的第三方库遵循各自许可证：

- **Qt**：[LGPL/GPL 商业许可](https://www.qt.io/licensing/)
- **OpenSceneGraph**：[LGPL/OSGPL](https://openscenegraph.com/)
- **PCL**：[BSD](https://github.com/PointCloudLibrary/pcl/blob/master/LICENSE.txt)
- **Eigen3**：[MPL2](https://eigen.tuxfamily.org/)
- **LASzip**：[LGPL](https://github.com/LASzip/LASzip)
- **nlohmann/json**：[MIT](https://github.com/nlohmann/json)

本项目自身的开源协议请参见仓库中的 `LICENSE` 文件（待补充）。
