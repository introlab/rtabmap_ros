cmake_minimum_required(VERSION 3.5)
project(rtabmap_launch)

find_package(ament_cmake REQUIRED)

install(DIRECTORY launch
   DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_cmake_pytest REQUIRED)
  ament_add_pytest_test(test_rtabmap_launch_args
    test/test_rtabmap_launch_args.py
    APPEND_ENV PYTHONPATH=${CMAKE_CURRENT_SOURCE_DIR}/launch
  )
endif()

ament_package()
