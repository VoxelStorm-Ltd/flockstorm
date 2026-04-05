cmake_minimum_required(VERSION 3.20)
project(flockstorm CXX)

set(CMAKE_CXX_STANDARD 23)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

# Silence FindBoost deprecation warning (CMake 3.27+: use BoostConfig.cmake)
if(POLICY CMP0167)
  cmake_policy(SET CMP0167 NEW)
endif()
# Allow FetchContent_Populate for repos that have no CMakeLists.txt (CMake 3.28+ deprecates it)
if(POLICY CMP0169)
  cmake_policy(SET CMP0169 OLD)
endif()

include(FetchContent)

# === Boost (required for hash in grid/base.h) ===
find_package(Boost REQUIRED)

# === VectorStorm (headers only, no CMakeLists.txt) ===
FetchContent_Declare(
  vectorstorm
  GIT_REPOSITORY https://github.com/VoxelStorm-Ltd/vectorstorm.git
  GIT_TAG        d2c319bb6d5ba47ddb0f5ad671a5ae9c920542d8
)
FetchContent_GetProperties(vectorstorm)
if(NOT vectorstorm_POPULATED)
  FetchContent_Populate(vectorstorm)
endif()

add_library(vectorstorm INTERFACE)
target_include_directories(vectorstorm INTERFACE ${vectorstorm_SOURCE_DIR})
target_link_libraries(vectorstorm INTERFACE Boost::boost)

# === MemoryStorm (has a .cpp file, no CMakeLists.txt) ===
FetchContent_Declare(
  memorystorm
  GIT_REPOSITORY https://github.com/VoxelStorm-Ltd/memorystorm.git
  GIT_TAG        89ccf180cf5f15847b25168feae9ef0e5daa8b0b
)
FetchContent_GetProperties(memorystorm)
if(NOT memorystorm_POPULATED)
  FetchContent_Populate(memorystorm)
endif()

add_library(memorystorm STATIC
  ${memorystorm_SOURCE_DIR}/memorystorm/memorystorm.cpp
)
target_include_directories(memorystorm PUBLIC ${memorystorm_SOURCE_DIR})

# === FlockStorm library ===
add_library(flockstorm STATIC
  flockstorm/manager.cpp
  flockstorm/grid/base.cpp
  flockstorm/grid/boid.cpp
  flockstorm/grid/get_cell.cpp
  flockstorm/obstacle/sphere.cpp
)
target_include_directories(flockstorm PUBLIC ${CMAKE_CURRENT_SOURCE_DIR})
target_link_libraries(flockstorm PUBLIC vectorstorm memorystorm)

# === Code coverage ===
option(FLOCKSTORM_COVERAGE "Enable gcov code coverage instrumentation" OFF)
if(FLOCKSTORM_COVERAGE)
  if(NOT CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang")
    message(WARNING "Coverage is only supported with GCC or Clang")
  endif()
  target_compile_options(flockstorm PRIVATE --coverage -O0 -g)
  target_link_options(flockstorm PRIVATE --coverage)
endif()

# === Tests ===
option(BUILD_TESTING "Build tests" ON)
if(BUILD_TESTING)
  enable_testing()
  add_subdirectory(tests)
endif()
