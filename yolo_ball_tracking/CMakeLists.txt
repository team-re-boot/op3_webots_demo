cmake_minimum_required(VERSION 3.5)
project(yolo_ball_tracking)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
  set(CMAKE_CXX_STANDARD_REQUIRED ON)
  set(CMAKE_CXX_EXTENSIONS OFF)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic -Werror)
endif()

find_package(backward_ros REQUIRED)
find_package(ament_cmake_auto REQUIRED)
ament_auto_find_build_dependencies()

ament_auto_add_executable(${PROJECT_NAME}_node
  src/${PROJECT_NAME}.cpp
)

add_backward(${PROJECT_NAME}_node)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

install(PROGRAMS
  scripts/image_converter.py
  DESTINATION lib/${PROJECT_NAME}
)

ament_auto_package(INSTALL_TO_SHARE launch)
