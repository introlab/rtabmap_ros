cmake_minimum_required(VERSION 3.5)
project(rtabmap_launch)

find_package(ament_cmake REQUIRED)

install(DIRECTORY launch
   DESTINATION share/${PROJECT_NAME}
)
# The launch file test, also run by rtabmap_examples and rtabmap_demos on their own
# launch files.
install(FILES test/launch_dry_run.py test/test_launch_files.py
   DESTINATION share/${PROJECT_NAME}/test
)

if(BUILD_TESTING)
  find_package(ament_cmake_pytest REQUIRED)
  # Dry-runs every installed launch file with each of its switches; nothing is started,
  # so it needs no DDS domain of its own. See test/launch_dry_run.py.
  ament_add_pytest_test(test_launch_files test/test_launch_files.py
    ENV RTABMAP_LAUNCH_TEST_PACKAGE=${PROJECT_NAME}
    TIMEOUT 900)
endif()

ament_package()
