cmake_minimum_required(VERSION 3.5)
project(rtabmap_demos)

find_package(ament_cmake REQUIRED)

install(DIRECTORY launch config params data
   DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_cmake_pytest REQUIRED)
  # Dry-runs every installed launch file with each of its switches; nothing is started,
  # so it needs no DDS domain of its own. The dry run comes from rtabmap_examples.
  ament_add_pytest_test(test_launch_files test/test_launch_files.py TIMEOUT 900)
  # Replays the demo bags through the demo launch files and compares the graphs with
  # test/golden. Skipped unless the bags were fetched (test/fetch_test_data.sh). The
  # demos' nodes talk over DDS, so it gets a domain of its own; see rtabmap_slam's for
  # the other packages' ranges.
  ament_add_pytest_test(test_demo_playback test/test_demo_playback.py
    ENV ROS_DOMAIN_ID=110
    TIMEOUT 3600)
endif()

ament_package()
