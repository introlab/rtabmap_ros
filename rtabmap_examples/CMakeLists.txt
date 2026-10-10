cmake_minimum_required(VERSION 3.5)
project(rtabmap_examples)

find_package(ament_cmake REQUIRED)

install(DIRECTORY launch config
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
endif()

ament_package()
