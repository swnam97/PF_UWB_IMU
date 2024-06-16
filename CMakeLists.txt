cmake_minimum_required(VERSION 3.10)
project(pf_uwb_imu)

# Find Python libraries
find_package(PythonLibs 3 REQUIRED)
find_package(PythonInterp 3 REQUIRED)
find_package(Eigen3 3.3 REQUIRED NO_MODULE)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  sensor_msgs
  std_msgs
  tf
  tf2_ros
  nlink_parser
)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES pf_uwb_imu
  CATKIN_DEPENDS roscpp sensor_msgs std_msgs tf nlink_parser
#  DEPENDS system_lib
)

include_directories(
# include
  ${catkin_INCLUDE_DIRS}
)

# Include directories for Python
include_directories(${PYTHON_INCLUDE_DIRS})
include_directories(${EIGEN3_INCLUDE_DIR})

# Include Matplotlib C++ header
set(MATPLOTLIBCPP_INCLUDE_DIR "/home/gihun/Downloads/matplotlib-cpp") # Adjust this path
include_directories(${MATPLOTLIBCPP_INCLUDE_DIR})

# Set source files
set(SRCS src/main.cpp src/particle_filter.cpp)
set_source_files_properties(${SRCS} PROPERTIES COMPILE_FLAGS -std=c++11)

# Create the executable
add_executable(particle_filter ${SRCS})
target_link_libraries(particle_filter ${PYTHON_LIBRARIES} ${catkin_LIBRARIES} Eigen3::Eigen /usr/lib/x86_64-linux-gnu/libpython3.8.so)  # Adjust the path as needed)

add_executable(path_tf_broadcaster src/path_tf_broadcaster.cpp)
target_link_libraries(path_tf_broadcaster
  ${catkin_LIBRARIES}
)

install(TARGETS path_tf_broadcaster
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)


# Conditional build for solution file
if(EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/src/particle_filter_sol.cpp")
    set(SOL_SRCS src/main.cpp src/particle_filter_sol.cpp)
    set_source_files_properties(${SOL_SRCS} PROPERTIES COMPILE_FLAGS -std=c++11)

    # Create the executable for the solution
    add_executable(particle_filter_solution ${SOL_SRCS})
    target_link_libraries(particle_filter_solution ${PYTHON_LIBRARIES} ${catkin_LIBRARIES} Eigen3::Eigen)
endif()
