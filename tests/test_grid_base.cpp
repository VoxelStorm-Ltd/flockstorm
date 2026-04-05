#include <catch2/catch_test_macros.hpp>
#include <catch2/catch_approx.hpp>
#include "flockstorm/grid/base.h"

TEST_CASE("grid::base - clear empties the grid", "[grid][base]") {
  flockstorm::grid::base g;
  g.grid[{0, 0, 0}].push_back(1);
  g.grid[{1, 1, 1}].push_back(2);
  REQUIRE(g.grid.size() == 2);
  g.clear();
  REQUIRE(g.grid.empty());
}

TEST_CASE("grid::base - clear is idempotent", "[grid][base]") {
  flockstorm::grid::base g;
  g.clear();
  g.clear();
  REQUIRE(g.grid.empty());
}

TEST_CASE("grid::base - default scale is 1.0", "[grid][base]") {
  flockstorm::grid::base g;
  REQUIRE(g.scale == Catch::Approx(1.0f));
}

TEST_CASE("grid::base - get_cell delegates correctly to the free function", "[grid][base]") {
  flockstorm::grid::base g;
  g.scale = 1.0f;

  SECTION("positive coordinates") {
    auto const cell = g.get_cell({2.5f, 3.5f, 4.5f});
    REQUIRE(cell.x == 2);
    REQUIRE(cell.y == 3);
    REQUIRE(cell.z == 4);
  }

  SECTION("negative coordinates") {
    auto const cell = g.get_cell({-0.1f, -1.0f, -2.9f});
    REQUIRE(cell.x == -1);
    REQUIRE(cell.y == -1);
    REQUIRE(cell.z == -3);
  }
}

TEST_CASE("grid::base - get_cell respects the scale member", "[grid][base]") {
  flockstorm::grid::base g;
  g.scale = 5.0f;
  auto const cell = g.get_cell({12.0f, 12.0f, 12.0f});
  // floor(12.0 / 5.0) == 2
  REQUIRE(cell.x == 2);
  REQUIRE(cell.y == 2);
  REQUIRE(cell.z == 2);
}

TEST_CASE("grid::base - grid entries can be added and retrieved", "[grid][base]") {
  flockstorm::grid::base g;
  vec3i const key{3, 4, 5};
  g.grid[key].push_back(42);
  g.grid[key].push_back(99);

  REQUIRE(g.grid.count(key) == 1);
  REQUIRE(g.grid[key].size() == 2);
  REQUIRE(g.grid[key][0] == 42);
  REQUIRE(g.grid[key][1] == 99);
}
