cmake_minimum_required(VERSION 3.5)
project(rtabmap_demos)

find_package(ament_cmake REQUIRED)

install(DIRECTORY launch config params data
   DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_cmake_pytest REQUIRED)
  # Dry-runs every installed launch file with each of its switches; nothing is started,
  # so it needs no DDS domain of its own. See test/launch_dry_run.py.
  ament_add_pytest_test(test_launch_files test/test_launch_files.py TIMEOUT 900)
endif()

ament_package()
