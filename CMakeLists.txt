cmake_minimum_required(VERSION 3.10)
project(ApplicationProject)

# ----------------- ADD THESE TWO LINES -----------------
# 1. Use the modern behavior for _ROOT variables (fixes the warning)
cmake_policy(SET CMP0074 NEW)

# 2. Tell CMake to look for modules in our new 'cmake' directory
list(APPEND CMAKE_MODULE_PATH "${CMAKE_SOURCE_DIR}/cmake")
# ---------------------------------------------------------

set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

# Use C++20 with warnings and debug symbols
set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
if(MSVC)
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} /W4 /Zi")
else()
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -g")
endif()

# Directories
set(APP_DIR ${CMAKE_SOURCE_DIR}/app)
set(SRC_DIR ${CMAKE_SOURCE_DIR}/modules)

# Sub-Directories
set(UTILS_DIR ${SRC_DIR}/utils)
set(MAPPER_DIR ${SRC_DIR}/Mapper/src)
set(ROUTING_DIR ${SRC_DIR}/Router/src)
set(BIN_DIR ${APP_DIR}/bin)
set(CONFIG_DIR ${APP_DIR}/input)
set(OR_UTILS_DIR ${SRC_DIR}/utils/milps)
set(TOPOLOGIES_DIR ${SRC_DIR}/utils/networks/topologies)

# Include paths
include_directories(
  "${OR_UTILS_DIR}"
  "${UTILS_DIR}"
  "${SRC_DIR}/Mapper/include"
  "${SRC_DIR}/Router/include"
  "${CMAKE_PREFIX_PATH}/include"
)

# Gather source files
file(GLOB APPLICATION_SRCS
  ${CMAKE_SOURCE_DIR}/app/src/Application.cpp
  ${UTILS_DIR}/nodes/Node.cpp
  ${MAPPER_DIR}/*.cpp
  ${ROUTING_DIR}/*.cpp
  ${CMAKE_SOURCE_DIR}/modules/utils/nodes/Router_node.cpp
)

file(GLOB TOPOLOGY_HELPERS
  ${TOPOLOGIES_DIR}/helper_function/*.cpp
)
list(APPEND APPLICATION_SRCS ${TOPOLOGY_HELPERS})

# Create binary output directory
file(MAKE_DIRECTORY ${BIN_DIR})

# Define the executable
add_executable(Application ${APPLICATION_SRCS})

target_include_directories(Application PRIVATE
  ${UTILS_DIR}/links/
)

# Place the executable in bin
set_target_properties(Application PROPERTIES
    RUNTIME_OUTPUT_DIRECTORY         ${BIN_DIR}/$<CONFIG>
    RUNTIME_OUTPUT_DIRECTORY_DEBUG   ${BIN_DIR}/Debug
    RUNTIME_OUTPUT_DIRECTORY_RELEASE ${BIN_DIR}/Release
)

# Find Python for post-run plotting
find_package(Python3 COMPONENTS Interpreter REQUIRED)

# Configuration filename (cache variable, can override via -DCONFIG=...)
set(CONFIG topology.config CACHE STRING "Configuration file name (from config directory)")

# Default run: uses topology.config
add_custom_target(run
  COMMAND $<TARGET_FILE:Application>  ${CONFIG_DIR}/${CONFIG}
  COMMAND ${Python3_EXECUTABLE}       ${CMAKE_SOURCE_DIR}/app/include/plot_unified.py
  DEPENDS Application
  WORKING_DIRECTORY ${CMAKE_SOURCE_DIR}
)

# Run all configs (except milp.config) sequentially
add_custom_target(run-all
  COMMAND $<TARGET_FILE:Application>  ${CMAKE_SOURCE_DIR}/app/include/config/butterfly.config
  COMMAND $<TARGET_FILE:Application>  ${CMAKE_SOURCE_DIR}/app/include/config/dragonfly.config
  COMMAND $<TARGET_FILE:Application>  ${CMAKE_SOURCE_DIR}/app/include/config/hypercube.config
  COMMAND $<TARGET_FILE:Application>  ${CMAKE_SOURCE_DIR}/app/include/config/mesh.config
  COMMAND $<TARGET_FILE:Application>  ${CMAKE_SOURCE_DIR}/app/include/config/ring.config
  COMMAND $<TARGET_FILE:Application>  ${CMAKE_SOURCE_DIR}/app/include/config/torus.config
  COMMAND $<TARGET_FILE:Application>  ${CMAKE_SOURCE_DIR}/app/include/config/tree.config
  COMMAND ${Python3_EXECUTABLE}       ${CMAKE_SOURCE_DIR}/app/include/plot_unified.py
  DEPENDS Application
  WORKING_DIRECTORY ${CMAKE_SOURCE_DIR}
)

add_custom_target(run-milp
  COMMAND $<TARGET_FILE:Application>  ${CONFIG_DIR}/${CONFIG} --milp
  COMMAND ${Python3_EXECUTABLE}       ${CMAKE_SOURCE_DIR}/app/include/plot_results.py
  DEPENDS Application
  WORKING_DIRECTORY ${CMAKE_SOURCE_DIR}
)
# OR-Tools Setup - Manual Linking
# Tell CMake where OR-Tools is
# list(APPEND CMAKE_PREFIX_PATH "D:/or-tools_x64_VisualStudio2022_cpp_v9.11.4210/or-tools_x64_VisualStudio2022_cpp_v9.11.4210")
# find_package(ortools REQUIRED)

# # # Link with OR-Tools target
# target_link_libraries(Application ortools::ortools)
# target_include_directories(Application PRIVATE
#   "D:/or-tools_x64_VisualStudio2022_cpp_v9.11.4210/or-tools_x64_VisualStudio2022_cpp_v9.11.4210/include"
# )
# OR-Tools Setup - Manual Linking
# Tell CMake where OR-Tools is
# list(APPEND CMAKE_PREFIX_PATH "/home/alhussein/data1/university/mart/ortools") # ortools built without cplex
list(APPEND CMAKE_PREFIX_PATH "/home/alhussein/data1/university/ortools-cplex") # ortools build with cplex


find_package(CPLEX REQUIRED)

find_package(ortools REQUIRED)

# # Link with OR-Tools target
target_link_libraries(Application ortools::ortools)
