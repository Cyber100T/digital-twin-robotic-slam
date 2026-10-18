# 毕设三种方法：代码与依赖库归档

提取日期：2026-10-18。范围依据你最终明确选择的 **ICP-LiDAR、FAST-LIO2、RTAB-Map**。

本目录包含从 Ubuntu 虚拟机和 Windows 原工程中提取的软件。未复制 rosbag、点云帧、图像帧、轨迹结果、地图、网格或 RTAB-Map 数据库。Notebook 的代码和文字单元保留，运行输出、嵌入附件及 widget 状态已从副本清除。原工程未修改。

## 从哪里开始

| 方法 | 代码入口 | 提取内容 |
|---|---|---|
| ICP-LiDAR | `code/windows/thesis_data/thesis_new21/slam_icp.ipynb` | 当前主版本、相同代码的其他实验目录副本、早期版本、导出脚本、可视化与 Poisson 网格重建辅助代码 |
| FAST-LIO2 | `code/ubuntu/fastlio_ws/src/FAST_LIO_ROS2/` | 源码、实际修改的 Mid360 配置与 RViz 配置、Livox 驱动、IMU 修正与地图保存包 |
| RTAB-Map | `code/ubuntu/rtabmap_viewer/scripts/` 和 `code/ubuntu/rtabmap_tools/` | 原始启动脚本各版本、RViz 配置、RGB 缩放、时间偏移、自动时间同步及 TF 发布代码 |

`code/windows/chalmers/thesis/slam_icp.ipynb` 是早期 ICP 实现。较新的 `thesis_new21`、`thesis_new2` 和 `thesis_test10` 中 ICP 核心代码相同，仍分别保留来源，未擅自替你删除版本。每个非空 Notebook 还附有按原单元顺序导出的 `.py`，方便阅读；这些 `.py` 保留原单元代码，不代表各单元已合并为经过测试的一键程序。

## Windows：ICP 代码和库

- 原代码位于 `C:\Users\22407\thesis_data` 与 `C:\Users\22407\chalmers\thesis`。
- 当前主 Notebook 记录 Python **3.11.15**，匹配本机 `E:\Anaconda3` 主环境；当前 `slam_o3d` 环境缺少 `tqdm`，未将它用作主要运行库来源。
- 已归档库版本：Open3D **0.19.0**、NumPy **1.26.4**、SciPy **1.17.0**、Matplotlib **3.10.8**、tqdm **4.67.3**。
- `libraries/windows_icp_runtime.zip` 保存解释器、标准库、ICP 和 Notebook 所需 Python 依赖及本机原生 DLL。软件自带的字体、渲染资源和配置属于运行库资源，不是实验数据。
- `environment/windows_icp_requirements_locked.txt` 是本次提取时的依赖版本清单；其余 `windows_*.json` 记录来源与筛选规则。

在 Windows 将运行库 ZIP 解压到独立目录后，可使用其中的 `python.exe -I -m jupyterlab` 打开 Notebook。原始脚本按相对路径查找输入，运行时仍需自行提供对应实验数据。本次没有执行建图或重新生成地图。

**保留的原始行为：**当前 ICP 主版本中 `USE_ODOM_PRIOR = True`、`USE_TF_STATIC = True`，会使用里程计先验和静态外参；归档保留这一实际实现，没有改成其他算法。若论文要描述为严格的 LiDAR-only 基线，需要另行核对这些开关与当时实验设置。

## Ubuntu：FAST-LIO2、RTAB-Map 和公共依赖

- 原系统：Ubuntu **22.04.5 LTS**，ROS 2 **Humble**，Python **3.10.12**，x86-64。
- RTAB-Map 是已安装的 ROS 二进制包，版本 **0.22.1**。虚拟机里未发现其完整上游源码仓库；已提取安装包中的库、头文件、启动配置，以及你的自定义代码。
- PCL **1.12.1**，OpenCV **4.5.4**，Eigen **3.4.0**。
- 公共支持源码还包括 `Livox-SDK2` 和 `unitree_ros2`，已排除构建缓存和用户实验数据。
- `libraries/ubuntu_runtime.tar.gz` 保存已安装的 ROS/RTAB-Map 相关运行文件、FAST-LIO2 编译安装产物、相关原生库和头文件，按原 Linux 路径布局保存。
- `environment/dependencies_locked.tsv` 保存选中依赖及其传递依赖的已安装版本；`dpkg_all.tsv` 和 `pip_freeze.txt` 为环境盘点记录。
- `environment/repositories.json` 保存仓库提交、上游地址、修改状态和修改补丁，保留你当时的本地配置。

FAST-LIO2 实际使用的源码配置为 `fastlio_ws/src/FAST_LIO_ROS2/config/mid360.yaml`。其中 IMU 主题是 `/imu_scaled`，需要配套的 `imu_scale_fix` 节点；源文件中的点云类型、标定矩阵等参数完整保留。

原启动入口保留在 `code/ubuntu/run_slam_all.sh`、`run_slam_all_2.sh`。RTAB-Map 的 `start_rtabmap_all_test3.sh` 使用自动时间同步；其他版本也一并保留。脚本中有原来的绝对路径和实验目录名，复用时需对应你的数据位置。

建议将运行库归档解压到独立检查目录，按版本清单准备 Ubuntu 环境后再恢复工程。该归档是软件快照，不能在 Windows 直接执行 Linux 库，也不等同于可任意迁移的完整虚拟机镜像。源代码恢复到 Ubuntu 的 `~/fastlio_ws/src` 后，可以在已配置 ROS Humble 及所列依赖的环境中重新构建。

## 校验与版本边界

`environment/source_manifest.json` 和 `windows_source_manifest.json` 记录文件来源及 SHA-256。`environment/runtime_manifest.json` 记录 Linux 库文件的校验值。`校验报告.json` 记录归档及代码完整性检查结果。

这些依赖版本来自本次提取时机器的当前状态。源码中的仓库提交与本地修改可追溯，但在没有旧环境锁文件的情况下，不能断言每个库版本都与毕业实验当天完全一致。由于按要求不提取数据，本次只检查代码与库的完整性、导入及链接依赖，没有重新进行三种方法的完整建图实验。
