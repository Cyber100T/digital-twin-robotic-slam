# Runtime archives

The software snapshots are stored locally under `libraries/` and are intended for separate release assets. They are excluded from Git commits. The source repository remains usable for inspection without downloading these archives.

## Windows

`windows_icp_runtime.zip` contains the selected Python 3.11 interpreter, standard library, mapping and notebook dependencies, MKL Python support modules, and native DLLs from the captured Anaconda environment. Unrelated Python packages, caches, tests, and examples were omitted.

The unpacked snapshot is checked using its own interpreter in an independent directory. This confirms that the tested Python libraries resolve inside that directory. It does not establish compatibility with every Windows installation or reproduce a mapping experiment without data.

Use the unpacked `python.exe -I -m jupyterlab` rather than assuming that old console launchers or a full Conda installation have been preserved. Native DLLs are retained as captured; this is a software snapshot rather than a freshly solved Conda environment.

## Ubuntu

`ubuntu_runtime.tar.gz` contains installed software files arranged beneath their original Linux paths, without leading slashes. Paths are normalized, and linked files were captured as regular-file contents. The archive includes ROS/RTAB-Map packages, FAST-LIO2 installation products, native dependencies, and selected headers.

Extract to an independent inspection directory first:

```bash
mkdir -p ~/thesis-runtime-inspect
tar -xzf ubuntu_runtime.tar.gz -C ~/thesis-runtime-inspect
```

Use matching Ubuntu/ROS packages and rebuild the workspace for a clean setup. The snapshot is not a full virtual-machine image, package-manager backup, or universally relocatable Linux application bundle. Some original scripts and compiled paths refer to `/home/jerry` and `/opt/ros/humble`.

## Checksums and provenance

`SHA256SUMS.txt` covers the delivered files, including both runtime archives. Source manifests retain original paths and hashes; Linux runtime entries have per-file hashes in `environment/runtime_manifest.json`.

The archive records the machine state at extraction on 2026-10-18. It does not assert that every installed version was unchanged since the thesis experiments.
