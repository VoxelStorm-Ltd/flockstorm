# Fetch Catch2
FetchContent_Declare(
  Catch2
  GIT_REPOSITORY https://github.com/catchorg/Catch2.git
  GIT_TAG        v3.7.1
)
FetchContent_MakeAvailable(Catch2)

add_executable(flockstorm_tests
  test_get_cell.cpp
  test_grid_base.cpp
  test_grid_boid.cpp
  test_obstacle_sphere.cpp
  test_manager.cpp
)

target_link_libraries(flockstorm_tests PRIVATE
  flockstorm
  Catch2::Catch2WithMain
)

if(FLOCKSTORM_COVERAGE)
  target_compile_options(flockstorm_tests PRIVATE --coverage -O0 -g)
  target_link_options(flockstorm_tests PRIVATE --coverage)
endif()

include(Catch)
catch_discover_tests(flockstorm_tests)
