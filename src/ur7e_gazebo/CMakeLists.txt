cmake_minimum_required(VERSION 3.8)
project(ur7e_gazebo)

find_package(ament_cmake REQUIRED)

install(
  DIRECTORY config launch worlds
  DESTINATION share/${PROJECT_NAME}
)

install(
  PROGRAMS scripts/wait_for_controllers.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME wait_for_controllers
)

install(
  FILES README.md
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_cmake_pytest REQUIRED)
  ament_add_pytest_test(test_gazebo_files test/test_gazebo_files.py)
endif()

ament_package()
