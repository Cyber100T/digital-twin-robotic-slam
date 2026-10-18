# Third-party software and provenance

This repository is a research software archive containing independently maintained components. It does not relicense upstream software. Preserve the original license, copyright, and notice files when using or redistributing any component.

| Component | Recovered location | Provenance |
|---|---|---|
| FAST-LIO ROS 2 port | `code/ubuntu/fastlio_ws/src/FAST_LIO_ROS2` | `Ericsii/FAST_LIO_ROS2`; commit and local patch recorded in `environment/repositories.json`; included `LICENSE` governs the recovered source |
| Livox ROS driver | `code/ubuntu/fastlio_ws/src/livox_ros_driver2` | `Livox-SDK/livox_ros_driver2`; included `LICENSE.txt` |
| Livox SDK2 | `code/ubuntu/Livox-SDK2` | `Livox-SDK/Livox-SDK2`; included `LICENSE.txt` and bundled dependency notices |
| Unitree ROS 2 | `code/ubuntu/unitree_ros2` | `unitreerobotics/unitree_ros2`; included `LICENSE` and Cyclone DDS notices |
| RTAB-Map | Installed Ubuntu packages and custom integration scripts | Upstream implementation was recovered as installed ROS packages rather than a full source checkout |
| Open3D, NumPy, SciPy, Matplotlib, Jupyter, and native dependencies | Separate runtime assets | Dependency versions and installed distribution metadata are retained in the environment records/runtime snapshots |

Repository provenance distinguishes upstream code from local modifications. In particular, the FAST-LIO configuration/RViz changes and Livox build change are preserved in the recovered source and patch records.

The thesis authors' custom notebooks, scripts, and configurations are included for research provenance. No blanket license has been assigned to those custom contributions in this archive. Contact the authors if you need a reuse grant beyond the applicable existing license terms.
