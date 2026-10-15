cmake_minimum_required(VERSION 3.8)
project(ncr_cbf)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(proxsuite REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)

add_library(ncr_cbf
    src/room.cpp
    src/pairwise.cpp
    src/saturation.cpp
    src/construct_constraints.cpp
    src/solver_wrapper.cpp
)

target_include_directories(ncr_cbf
    PUBLIC
        ${CMAKE_CURRENT_SOURCE_DIR}/include
)

target_link_libraries(ncr_cbf
    PUBLIC
        Eigen3::Eigen
        proxsuite::proxsuite
)

add_executable(ncr_cbf_node
    src/ncr_cbf_node.cpp
)

target_link_libraries(ncr_cbf_node
    PRIVATE
        ncr_cbf
)

ament_target_dependencies(ncr_cbf_node
    rclcpp
    geometry_msgs
)

install(TARGETS ncr_cbf ncr_cbf_node
    ARCHIVE DESTINATION lib
    LIBRARY DESTINATION lib
    RUNTIME DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY param/
    DESTINATION share/${PROJECT_NAME}/param
)

add_executable(test_room
    test/test_room.cpp
)

target_link_libraries(test_room
    PRIVATE
        ncr_cbf
)

add_executable(test_pairwise
    test/test_pairwise.cpp
)

target_link_libraries(test_pairwise
    PRIVATE
        ncr_cbf
)

add_executable(test_saturation
    test/test_saturation.cpp
)

target_link_libraries(test_saturation
    PRIVATE
        ncr_cbf
)

add_executable(test_construct_constraints
    test/test_construct_constraints.cpp
)

target_link_libraries(test_construct_constraints
    PRIVATE
        ncr_cbf
)

add_executable(test_solver_wrapper
    test/test_solver_wrapper.cpp
)

target_link_libraries(test_solver_wrapper
    PRIVATE
        ncr_cbf
)

enable_testing()

add_test(
    NAME test_room
    COMMAND test_room
)

add_test(
    NAME test_pairwise
    COMMAND test_pairwise
)

add_test(
    NAME test_saturation
    COMMAND test_saturation
)

add_test(
    NAME test_construct_constraints
    COMMAND test_construct_constraints
)

add_test(
    NAME test_solver_wrapper
    COMMAND test_solver_wrapper
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
