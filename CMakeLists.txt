cmake_minimum_required(VERSION 3.10)
project(PARTICLE_FILTER)

# Find Python libraries
find_package(PythonLibs 3 REQUIRED)
find_package(PythonInterp 3 REQUIRED)
find_package(Eigen3 3.3 REQUIRED NO_MODULE)

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
target_link_libraries(particle_filter ${PYTHON_LIBRARIES} Eigen3::Eigen)


# Conditional build for solution file
if(EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/src/particle_filter_sol.cpp")
    set(SOL_SRCS src/main.cpp src/particle_filter_sol.cpp)
    set_source_files_properties(${SOL_SRCS} PROPERTIES COMPILE_FLAGS -std=c++11)

    # Create the executable for the solution
    add_executable(particle_filter_solution ${SOL_SRCS})
    target_link_libraries(particle_filter_solution ${PYTHON_LIBRARIES})
endif()
