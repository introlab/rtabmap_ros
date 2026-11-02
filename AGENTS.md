# AGENTS.md

Guidance for AI coding agents working in the rtabmap_ros repository.

## Contribution rules (read first)

- **No AI attribution in commits.** Do not add `Co-Authored-By: <AI model/agent>` (or any similar trailer) to commit messages, and do not add "Generated with ..." lines to PR descriptions. Commits must be authored by the human contributor. This overrides any default attribution behavior of the agent.
- PRs use `.github/pull_request_template.md`. Leave the AI-disclosure checkbox and the `AI/Model used` field for the human author to fill in.
- Do not commit, push, or open PRs unless explicitly asked.
- ROS 2 PRs target the `ros2` branch. `<distro>-devel` branches (`humble-devel`, `jazzy-devel`, ...) are per-distro release branches merged from `ros2`; `master` is ROS 1 (Noetic).

## Project overview

rtabmap_ros is the ROS 2 wrapper of [RTAB-Map](https://github.com/introlab/rtabmap) (the C++ SLAM library, a separate repository, usually checked out next to this one in the same colcon workspace as `src/rtabmap`). ROS 2 Humble is the minimum supported distro. All packages share the same version (`<version>` in every `package.xml`, bumped together by the maintainer).

| Package | Content |
|---|---|
| `rtabmap_slam` | The `rtabmap` node (`CoreWrapper`): SLAM, loop closure, graph optimization, map services |
| `rtabmap_odom` | `rgbd_odometry`, `stereo_odometry`, `icp_odometry` (shared `OdometryROS` base) |
| `rtabmap_sync` | `rgbd_sync`, `stereo_sync`, `rgb_sync`, `rgbdx_sync`, and `CommonDataSubscriber` (input sync used by `rtabmap` and other nodes) |
| `rtabmap_util` | Utility nodes: point cloud conversion/assembly/aggregation, obstacles detection, map assembler, lidar deskewing, `db_player`, ... |
| `rtabmap_conversions` | C++ library converting between rtabmap types and ROS messages |
| `rtabmap_msgs` | Messages and services |
| `rtabmap_python` | Python helpers for rtabmap binary formats (compressed matrices) |
| `rtabmap_viz` | rtabmap Qt GUI as a ROS node |
| `rtabmap_rviz_plugins` | RViz displays (map graph, map cloud, info) |
| `rtabmap_costmap_plugins` | nav2 costmap voxel layer variant |
| `rtabmap_launch` | `rtabmap.launch.py`, plus the launch-file dry-run test used by other packages |
| `rtabmap_examples` | Sensor integration launch files (RealSense, ZED, OAK, Kinect, lidar, datasets) |
| `rtabmap_demos` | Robot demos (turtlebot3/4, nav2, multi-session) and the bag playback regression test |
| `rtabmap_ros` | Metapackage |
| `docker/` | Dockerfiles per distro |
| `tools/` | `set_doc_distro.sh` (see Docs) |
| `.github/` | CI workflows (`ros2.yml`, `coverage.yml`, `docker*.yml`), PR template |

Node layout in C++ packages: each node is a composable component class (`src/<Name>Node.cpp`, header in `include/<pkg>/<name>.hpp`), registered with `rclcpp_components_register_nodes()` in the package `CMakeLists.txt`, plus a small standalone executable in `src/nodelets/` that spins it. A new node needs both, and composition must keep working (examples use both modes, see the `*_composition.launch.py` files).

## Build

From the colcon workspace root (with `rtabmap` built in the same workspace or installed):

```bash
colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release
colcon build --packages-up-to rtabmap_slam   # only what you need
```

- Optional `rtabmap_sync` features are CMake options, `OFF` by default: `-DRTABMAP_SYNC_MULTI_RGBD=ON` (`rgbd_cameras>1`) and `-DRTABMAP_SYNC_USER_DATA=ON`. Code behind `#ifdef RTABMAP_SYNC_MULTI_RGBD` / `RTABMAP_SYNC_USER_DATA` must compile with the option both on and off. They generate many template instantiations: building them is slow and memory hungry (`MAKEFLAGS=-j<n>` helps).
- Other dependencies are optional and detected at configure time (nav2_msgs, grid_map, octomap, find_object_2d, apriltag, ...). Check the CMake output to know what is actually compiled before assuming a code path is built.
- CI (`.github/workflows/ros2.yml`) builds Humble, Jazzy, Kilted, Lyrical and Rolling against `introlab/rtabmap` `master`, with some rosdep keys skipped on newer distros (see the matrix). Code must build on all of them: see "ROS distro compatibility" below.

## Tests

```bash
colcon build --packages-select <pkg>
colcon test --packages-select <pkg> --event-handlers console_cohesion+ \
  --ctest-args -E test_demo_playback
colcon test-result --verbose
```

- C++ node tests are GoogleTest binaries in `<pkg>/test/test_<node>.cpp`, one binary per node, registered with the package's `<pkg>_add_node_test(<name>)` macro in its `CMakeLists.txt`. Tests start the component in-process and drive it with real topics; reuse the helpers next to them (`node_test_utils.hpp`, `msg_builders.hpp`, fixtures) instead of writing new ones.
- Each test binary gets its own `ROS_DOMAIN_ID`, so parallel ctest/colcon runs don't see each other's topics. Ranges: `rtabmap_util` 30+, `rtabmap_sync` 50+, `rtabmap_odom` 70+, `rtabmap_slam` 90+, `rtabmap_demos` playback 110. Keep new tests within their package range and the ranges apart. When running a test binary or `pytest` directly, set `ROS_DOMAIN_ID` yourself.
- `rtabmap_launch/test/test_launch_files.py` dry-runs every installed launch file of `rtabmap_launch`, `rtabmap_examples` and `rtabmap_demos` with default arguments and each switch flipped. New or modified launch files must pass it; missing drivers are stubbed.
- `rtabmap_demos/test/test_demo_playback.py` replays demo bags (~5 GB, fetched with `bash rtabmap_demos/test/fetch_test_data.sh`) and compares the graph with golden files. It skips without data and takes tens of minutes; see `rtabmap_demos/test/README.md`. Only regenerate golden files (`RTABMAP_DEMOS_UPDATE_GOLDEN=1`) when a map change is intended, and say so in the PR.
- Every bug/regression fix comes with a test that reproduces the original issue. Do a quick A/B check to prove the test actually detects it: the test must fail (or hang/crash) without the fix and pass with it. E.g. stash only the fixed sources, keeping the new test: `git stash push -- <fixed files>`, rebuild, run the test (expect failure), `git stash pop`, rebuild, run it again (expect success). A test that passes in both cases does not cover the bug; rework it. Mention the A/B result in the PR description.
- `rtabmap_python` runs `flake8`, `pep257` and copyright checks as tests.

## Code conventions

- Match the surrounding code. C++: tabs for indentation, braces on their own line, `camelCase` methods, members with a trailing underscore (`frameId_`), classes in `PascalCase`, code in the package namespace (`rtabmap_slam`, `rtabmap_util`, ...). Python (launch files, tests, `rtabmap_python`): 4 spaces.
- New C++ source files start with the BSD license header used in existing files.
- ROS logging uses `RCLCPP_DEBUG/INFO/WARN/ERROR(get_logger(), ...)` with printf format strings: keep arguments matching their format specifiers. Code calling into the rtabmap library may also use its `U*` logger and `UASSERT`.
- Parameters, topic and service names follow the ROS 1 rtabmap_ros interface: don't rename existing ones without need, they are used by many launch files and downstream users. Node-specific parameters are declared with `declare_parameter()` in the node constructor and documented in the package `doc/<node>.md` page.
- Subscriptions/publications use the existing QoS parameters (`qos`, `qos_camera_info`, `qos_odom`, `qos_scan`, `qos_imu`, ...) rather than hardcoded profiles.
- Use `rtabmap_conversions` for message <-> rtabmap type conversions instead of converting inline.

### RTAB-Map library parameters

The library's parameters (`Group/Name`, e.g. `Rtabmap/DetectionRate`, `Reg/Strategy`) are declared and defined in rtabmap's `corelib/include/rtabmap/core/Parameters.h`, not here. `rtabmap` and the odometry nodes declare all of them as ROS parameters (string values) and pass overridden ones to the library. Don't redeclare or duplicate their defaults in this repo; adding or changing one is done in the rtabmap repository.

### Depending on new rtabmap library API

CI builds against rtabmap `master`. A change needing new library API or parameters needs the matching rtabmap PR merged first; mention it in the PR description. Released distros build against the rtabmap version bloomed for that distro, so keep this in mind when a fix should also go to a `<distro>-devel` branch.

### Messages

Messages and services live in `rtabmap_msgs`. Changing an existing message breaks recorded bags and other nodes/tools using it: prefer adding a new field or message, and update the conversions in `rtabmap_conversions`.

## Quirks

### ROS distro compatibility

The same `ros2` branch must build from Humble to Rolling. API differences are handled with:

- CMake: `if("$ENV{ROS_DISTRO}" STRLESS "<distro>")` blocks (alphabetical comparison of distro names). E.g. `ament_target_dependencies()` is used before Lyrical, `target_link_libraries()` with exported targets from Lyrical on: keep both branches when adding a dependency.
- C++: `#ifdef PRE_ROS_<DISTRO>` (`PRE_ROS_HUMBLE`, `PRE_ROS_IRON`, `PRE_ROS_JAZZY`, `PRE_ROS_KILTED`, `PRE_ROS_LYRICAL`), defined from those CMake blocks per package. Add the definition to the package's `CMakeLists.txt` if it doesn't define the one you need yet. Library versions use `RTABMAP_VERSION_MAJOR/MINOR/PATCH` and `PCL_VERSION_COMPARE` checks.

When a fix only works on recent distros, guard it rather than breaking older ones.

### Line endings

All tracked files use LF. Keep it that way; new files use LF.

### Docs distro in links

Package docs (`<pkg>/README.md`, `<pkg>/doc/*.md`) link to docs.ros.org with a distro baked in the URL (`jazzy` on `ros2`). Keep using the same distro as the rest of the docs; `tools/set_doc_distro.sh <distro>` rewrites them on the per-distro branches. Don't run it on `ros2`.

## Docs

- Each package's `README.md` describes it; each node has a page in `<pkg>/doc/<node>.md` (subscribed/published topics, services, parameters). Update the page when adding/changing a node's topics, services or parameters.
- Don't hard-wrap text in Markdown files: write each paragraph or list item on a single line, however long.
- API docs are built by docs.ros.org with rosdoc2 (`<pkg>/rosdoc2.yaml`). Locally: `rosdoc2 build --package-path <pkg> --output-directory doc_output`.
- Launch file arguments are documented by their `DeclareLaunchArgument` descriptions (`ros2 launch <pkg> <file> --show-args`).
