cmake_minimum_required(VERSION 3.10)

project(g2o)

# =========================
# DEPENDENCIES
# =========================

find_package(Eigen3 REQUIRED)

# =========================
# GLOBAL FLAGS (ARM64 SAFE)
# =========================

set(CMAKE_POSITION_INDEPENDENT_CODE ON)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

option(G2O_USE_OPENMP "Enable OpenMP support" OFF)

# =========================
# OPENMP (OPTIONAL SAFE)
# =========================

find_package(OpenMP)

if(OpenMP_FOUND AND G2O_USE_OPENMP)
  add_definitions(-DUSE_OPENMP)
  message(STATUS "g2o: OpenMP enabled")
else()
  message(STATUS "g2o: OpenMP disabled")
endif()

# =========================
# DEFINES
# =========================

if(UNIX)
  add_definitions(-DUNIX)
endif()

# =========================
# OUTPUT DIR (ROS2 SAFE)
# =========================

set(CMAKE_LIBRARY_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/lib)
set(CMAKE_ARCHIVE_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/lib)
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/bin)

# =========================
# INCLUDE DIRS
# =========================

set(G2O_ROOT ${CMAKE_CURRENT_SOURCE_DIR})

include_directories(
  ${G2O_ROOT}
  ${G2O_ROOT}/g2o/core
  ${G2O_ROOT}/g2o/types
  ${G2O_ROOT}/g2o/stuff
  ${EIGEN3_INCLUDE_DIRS}
)

# =========================
# SOURCE FILES (IMPORTANT FIX)
# =========================

file(GLOB_RECURSE G2O_CORE_SRC
  "${G2O_ROOT}/g2o/core/*.cpp"
)

file(GLOB_RECURSE G2O_TYPES_SRC
  "${G2O_ROOT}/g2o/types/*.cpp"
)

file(GLOB_RECURSE G2O_STUFF_SRC
  "${G2O_ROOT}/g2o/stuff/*.cpp"
)

set(G2O_SOURCES
  ${G2O_CORE_SRC}
  ${G2O_TYPES_SRC}
  ${G2O_STUFF_SRC}
)

# =========================
# LIBRARY
# =========================

add_library(g2o SHARED ${G2O_SOURCES})

target_link_libraries(g2o Eigen3::Eigen)

target_include_directories(g2o PUBLIC
  ${G2O_ROOT}
  ${G2O_ROOT}/g2o/core
  ${G2O_ROOT}/g2o/types
  ${G2O_ROOT}/g2o/stuff
  ${EIGEN3_INCLUDE_DIRS}
)

# =========================
# INSTALL (ROS2 SAFE)
# =========================

install(TARGETS g2o
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
  RUNTIME DESTINATION lib
)
