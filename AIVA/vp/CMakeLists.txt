cmake_minimum_required(VERSION 3.8)
project(vp)

# Compiler Flags
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find Dependencies
find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(builtin_interfaces REQUIRED)

# Generate Interfaces
rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/VitalPulse.msg"
  DEPENDENCIES builtin_interfaces
)

# Testing (Optional)
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

# Export Dependencies (Crucial for downstream use)
ament_export_dependencies(rosidl_default_runtime)

# Finalize Package
ament_package()
