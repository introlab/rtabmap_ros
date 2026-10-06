cmake_minimum_required(VERSION 3.5)
project(rtabmap_demos)

find_package(ament_cmake REQUIRED)

install(DIRECTORY launch config params data
   DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_cmake_pytest REQUIRED)
  # rtabmap_launch's launch file test, on this package's installed launch files: each
  # dry-run with each of its switches; nothing is started, so it needs no DDS domain of its
  # own. Run from where rtabmap_launch installed it: no pytest cache written there.
  find_package(rtabmap_launch REQUIRED)
  ament_add_pytest_test(test_launch_files ${rtabmap_launch_DIR}/../test/test_launch_files.py
    ENV RTABMAP_LAUNCH_TEST_PACKAGE=${PROJECT_NAME} "PYTEST_ADDOPTS=-p no:cacheprovider"
    TIMEOUT 900)
  # Replays the demo bags through the demo launch files and compares the graphs with
  # test/golden. Skipped unless the bags were fetched (test/fetch_test_data.sh). The
  # demos' nodes talk over DDS, so it gets a domain of its own; see rtabmap_slam's for
  # the other packages' ranges.
  ament_add_pytest_test(test_demo_playback test/test_demo_playback.py
    ENV ROS_DOMAIN_ID=110
    TIMEOUT 3600)
endif()

ament_package()
