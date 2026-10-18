# Digital Twin Generation via Robotic SLAM for Wireless Ray-Tracing Applications

Research software accompanying the 2026 master's thesis by **Jiarui He and Yuzheng Long**, Department of Electrical Engineering, Chalmers University of Technology.

This repository preserves the three reconstruction methods evaluated in the thesis: **ICP-based LiDAR mapping, FAST-LIO2, and RTAB-Map**. It contains the recovered source code, experiment configuration, supporting tools, and dependency records. Recorded sensor data and reconstructed results are excluded.

[Release downloads](https://github.com/Cyber100T/digital-twin-robotic-slam/releases/tag/v2026.10.18) · [Reproduction guide](docs/REPRODUCIBILITY.md) · [Third-party software](THIRD_PARTY.md)

## Research context

The thesis develops an indoor wireless digital twin workflow using a Unitree Go2 robot equipped with a Livox MID-360 LiDAR and an Intel RealSense D435i RGB-D camera. SLAM produces the geometric map; subsequent point cloud processing, surface reconstruction, Blender editing, material assignment, and Mitsuba/XML export prepare the scene for Sionna RT.

```mermaid
flowchart LR
    A[Recorded ROS 2 sensor data] --> B[ICP / FAST-LIO2 / RTAB-Map]
    B --> C[Registered point cloud]
    C --> D[Filtering and surface reconstruction]
    D --> E[Blender scene and radio materials]
    E --> F[Mitsuba XML / Sionna RT]
```

**Repository scope:** the mapping components and recovered preprocessing, meshing, and visualization tools. The diagram describes the full thesis workflow; this software archive does not contain the complete downstream Blender/Sionna project or simulation results.

## Methods

| Method | Role in the thesis | Recovered implementation |
|---|---|---|
| ICP-based LiDAR mapping | Geometric registration baseline used to inspect drift and scan-layer artifacts | Windows Open3D notebooks, an earlier baseline, data export tools, and visualization code |
| FAST-LIO2 | Main LiDAR-inertial reconstruction method selected for subsequent digital twin generation | ROS 2 source tree, Livox driver, actual MID-360 configuration, IMU scaling node, map saver, and launch scripts |
| RTAB-Map | RGB-D reconstruction comparison, including coverage and geometric completeness | Custom RGB-D/point-cloud launch configurations, time synchronization, odometry-to-TF tools, RViz configuration, and installed package records |

The thesis selects FAST-LIO2 for its useful geometric coverage and map continuity under the recorded experimental conditions. This repository does not claim a general SLAM ranking or provide a rerun of the thesis experiments. See Chapter 5, Table 5.1, and Sections 8.1.2-8.1.5 of the thesis.

### Implementation details that affect interpretation

- The recovered later ICP notebook enables `USE_ODOM_PRIOR` and `USE_TF_STATIC`. It therefore supports external odometry initialization and static extrinsics. The earlier ICP notebook is also retained. Inspect these settings before describing a recovered run as strictly LiDAR-only.
- Later RTAB-Map launch variants use RGB-D streams, Livox point clouds, and synchronized robot odometry; their internal visual and ICP odometry options are disabled. These configurations are preserved as recovered, rather than rewritten to match a generic RGB-D example.
- FAST-LIO2's archived `mid360.yaml` subscribes to `/imu_scaled`, making the included IMU scaling node part of that configuration.

## Repository layout

```text
code/
  windows/
    thesis_data/
      thesis_new21/                 # Later ICP notebook and visualization
      thesis_new2/                  # Preserved experiment code copy
      thesis_test10/                # ICP and Poisson meshing notebooks
    chalmers/thesis/                 # Earlier ICP baseline
  ubuntu/
    fastlio_ws/src/
      FAST_LIO_ROS2/                # FAST-LIO2 source and configuration
      livox_ros_driver2/            # Livox ROS driver
      imu_scale_fix/                # IMU unit conversion
      map_saver/                    # Map export node
    rtabmap_tools/                  # Synchronization, resize, and TF utilities
    rtabmap_viewer/                 # Launch scripts and RViz configuration
    Livox-SDK2/                     # Supporting SDK source
    unitree_ros2/                   # Robot interfaces and supporting source
environment/                        # Versions, provenance, and file manifests
docs/                               # Reproduction and archive documentation
libraries/                          # Local runtime archives; distributed separately
```

Notebook cell sources are retained. Execution outputs, attached images, and widget state were removed from the exported copies. Nonempty notebooks also have `.py` reading copies with their original cell boundaries. The original files on the host and virtual machine were not edited.

## Software environment

| Component | Captured version |
|---|---|
| Ubuntu / ROS 2 | Ubuntu 22.04.5 LTS / Humble |
| Ubuntu Python | 3.10.12 |
| RTAB-Map | 0.22.1 |
| PCL / OpenCV / Eigen | 1.12.1 / 4.5.4 / 3.4.0 |
| Windows Python | 3.11.15 |
| Open3D | 0.19.0 |
| Windows NumPy / SciPy | 1.26.4 / 1.17.0 |
| Windows Matplotlib / tqdm | 3.10.8 / 4.67.3 |

These are **capture-time versions from 18 October 2026**. The later ICP notebook records Python 3.11.15, matching the selected Windows environment. Exact historical package versions at the time of each thesis experiment cannot be inferred from notebook metadata alone.

Detailed records are in [`environment/dependencies_locked.tsv`](environment/dependencies_locked.tsv), [`environment/windows_icp_requirements_locked.txt`](environment/windows_icp_requirements_locked.txt), and [`environment/repositories.json`](environment/repositories.json).

## Getting started

For **ICP**, open [`slam_icp.ipynb`](code/windows/thesis_data/thesis_new21/slam_icp.ipynb) with the Windows environment. Review the initial configuration cell and provide your own input files. Expected inputs include `lidar_frames/`, `lidar_frames/timestamps.csv`, and, when enabled, `odom.csv` and `tf_static.json`.

For **FAST-LIO2**, restore `code/ubuntu/fastlio_ws` as `~/fastlio_ws` in a compatible Ubuntu/ROS environment, install the dependencies listed in the environment records, and build the workspace. The normal node startup is:

```bash
# Source the ROS and rebuilt workspace environments in each terminal.
source /opt/ros/humble/setup.bash
source ~/fastlio_ws/install/setup.bash

# Terminal 1
ros2 run imu_scale_fix imu_scale_node

# Terminal 2, after sourcing both environments again
ros2 launch fast_lio mapping.launch.py
```

Replay your rosbag in a separate terminal after reviewing its topics, timestamps, and frame relationships. The thesis used reduced playback speed, typically `0.5`, for the FAST-LIO2 workflow. The original orchestration scripts are preserved as [`run_slam_all.sh`](code/ubuntu/run_slam_all.sh) and [`run_slam_all_2.sh`](code/ubuntu/run_slam_all_2.sh).

For **RTAB-Map**, inspect [`start_rtabmap_all_test3.sh`](code/ubuntu/rtabmap_viewer/scripts/start_rtabmap_all_test3.sh) and the earlier variants. Restore the tools to their expected home-directory locations and adapt recording paths and calibration. See the [reproduction guide](docs/REPRODUCIBILITY.md) before running these scripts.

## Data and runtime archives

No rosbag recordings, image sequences, point cloud datasets, trajectories, reconstructed maps, meshes, or RTAB-Map databases are included. Inputs must be supplied separately.

Large runtime snapshots are kept outside Git history:

| Asset | Contents |
|---|---|
| `windows_icp_runtime.zip` | Selected Windows interpreter, standard library, Python packages, and native runtime DLLs |
| `ubuntu_runtime.tar.gz` | Installed ROS/RTAB-Map dependencies, FAST-LIO2 installation products, supporting native libraries, and headers |

The RTAB-Map upstream source was not present in the recovered virtual machine; its installed software is preserved in the runtime snapshot. See [archive notes](docs/RUNTIME_ARCHIVES.md) for verification and restoration boundaries.

## Verification

The extraction was checked against per-file SHA-256 manifests. Linux executable and shared-library dependencies were inspected in the ROS environment, with the OGRE plugin directory added for plugin checks. The Windows runtime was unpacked to an independent directory and checked for local imports of the required mapping and notebook libraries.

See [`校验报告.json`](校验报告.json) and [`SHA256SUMS.txt`](SHA256SUMS.txt). No sensor-data replay or complete mapping experiment was performed during extraction.

## Contributors

- **Jiarui He** ([SiLenTBlizZ](https://github.com/SiLenTBlizZ)) - thesis co-author.
- **Yuzheng Long** ([Yuzheng1ong](https://github.com/Yuzheng1ong)) - thesis co-author.

Both authors are credited for the thesis research software preserved in this repository. Upstream components retain their original authorship and attribution.

## Citation and third-party attribution

If this software supports your work, cite:

> Jiarui He and Yuzheng Long. *Digital Twin Generation via Robotic SLAM for Wireless Ray-Tracing Applications*. Master's thesis, Chalmers University of Technology, 2026.

Machine-readable software citation information is provided in [`CITATION.cff`](CITATION.cff). Upstream software retains its own licenses and attribution; see [`THIRD_PARTY.md`](THIRD_PARTY.md). No single repository-wide license is assigned to this collection of independently licensed components.
