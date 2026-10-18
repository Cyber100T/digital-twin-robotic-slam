# Reproduction guide

This guide distinguishes recovered code from steps needed to prepare a new machine. Paths and calibration values in the original code were deliberately preserved.

## 1. Prepare inputs

The thesis platform used a Unitree Go2, Livox MID-360, and RealSense D435i. The recorded ROS 2 topics included:

| Input | Topics |
|---|---|
| LiDAR and IMU | `/livox/lidar`, `/livox/imu` |
| Robot state | `/utlidar/robot_odom`, `/utlidar/robot_pose`, `/utlidar/imu` |
| RGB-D | `/camera/color/image_raw`, `/camera/color/camera_info`, `/camera/depth/image_rect_raw`, `/camera/depth/camera_info`, `/camera/imu` |
| Transforms | `/tf`, `/tf_static` |

Only a method's required inputs need to be provided. Check `ros2 bag info` for topic types and verify timestamp and TF consistency before playback. Do not reuse an old time offset without checking the new recording.

## 2. Windows ICP

The selected environment was captured from `E:\Anaconda3`. Install the packages from `environment/windows_icp_requirements_locked.txt` in a compatible Python 3.11 environment, or use the separate Windows runtime snapshot. For the snapshot, unpack it to a new directory and invoke that directory's interpreter:

```powershell
& 'C:\path\to\unpacked-runtime\python.exe' -I -m jupyterlab
```

Open `code/windows/thesis_data/thesis_new21/slam_icp.ipynb`. Run it with the working directory containing the input files expected by its first code cell. The example path above is a location to choose on your machine, not a directory included in the repository.

The later notebook reads:

- `lidar_frames/*.npy` and `lidar_frames/timestamps.csv`;
- `odom.csv` when odometry priors are enabled;
- `tf_static.json` when static-transform support is enabled.

Review `FRAME_ACCUM`, voxel sizes, range/height filters, and coarse/fine ICP thresholds. Preserve a record of the two prior-related switches for any comparison with the thesis's LiDAR-only baseline. The earlier `code/windows/chalmers/thesis/slam_icp.ipynb` is a separate recovered implementation.

Data export code is retained alongside the experiment notebooks. `poisson_mesh.ipynb` in `thesis_test10` is a recovered surface-reconstruction utility. It requires a point cloud input supplied separately.

## 3. Ubuntu FAST-LIO2

Use an Ubuntu 22.04 x86-64 environment with ROS 2 Humble and the dependencies recorded in `environment/`. The archive includes dependency records, not a new operating-system installer.

1. Copy the recovered `code/ubuntu/fastlio_ws` directory to `~/fastlio_ws`.
2. Make the Livox SDK available, using the included SDK source or a matching installed version.
3. Source ROS Humble and rebuild the workspace:

```bash
source /opt/ros/humble/setup.bash
cd ~/fastlio_ws
colcon build --symlink-install
source install/setup.bash
```

4. Inspect `src/FAST_LIO_ROS2/config/mid360.yaml`. The captured settings include `/livox/lidar`, `/imu_scaled`, `lidar_type: 4`, the original extrinsic matrix, and the original map-output path.
5. Start `imu_scale_fix` and `fast_lio` in separate sourced terminals, as shown in the main README.
6. Replay a recording using the appropriate clock and simulation-time settings for the launch configuration. The source orchestration scripts preserve the original playback commands, including the reduced rate of `0.5`.
7. Inspect the map in RViz and invoke the included map saver when appropriate:

```bash
ros2 run map_saver save_laser_map
```

Check output paths before using the saver. The archive does not contain the original output PCD files.

## 4. Ubuntu RTAB-Map

Restore `rtabmap_tools` and `rtabmap_viewer` under the intended user's home directory, or update their references consistently. Install compatible RTAB-Map ROS packages using the recorded versions as a reference.

The recovered launch variants are preserved to show the progression of the setup:

| Script | Main supporting behavior |
|---|---|
| `start_rtabmap_all.sh` | Initial launch and static transforms |
| `start_rtabmap_all_test.sh` | Fixed odometry timestamp offset |
| `start_rtabmap_all_test2.sh` | Timestamp offset with odometry-to-TF publication |
| `start_rtabmap_all_test3.sh` | Automatic synchronization and TF publication; bag TF remapping |

Inspect the selected script before execution. These are the original desktop orchestration scripts: they open terminal windows, stop matching ROS/RViz processes, and pass `--delete_db_on_start` to RTAB-Map. Use a separate working session and database location when retaining a previous RTAB-Map reconstruction.

Update bag paths, frames, camera calibration, and topic selections for your recording. Later variants consume `/utlidar/robot_odom_synced` and disable the internal visual/ICP odometry nodes. RGB scaling, fixed-offset timing, and automatic synchronization tools are available in `rtabmap_tools`; enable only the tools required by the selected launch.

## 5. Evaluation and downstream scene preparation

The thesis evaluates maps for coverage, local alignment, visible drift, point density, noise/artifacts, and meshing suitability. Its later geometry validation compares measurable dimensions in the real scene and reconstructed mesh.

For the complete thesis workflow, maps must still undergo filtering, surface reconstruction, mesh cleanup, coordinate checking, material assignment, and compatible scene export before Sionna RT simulation. A point cloud alone is not the final ray-tracing scene. Downstream scene assets and simulation notebooks are outside this three-method archive.

## Validation boundary

The archived source and runtime contents have extraction-integrity checks. Successful library import/link checks do not replace compiling on a new host, replaying a recording, validating calibration, or reproducing the thesis's reconstruction results.
