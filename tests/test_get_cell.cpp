#include <catch2/catch_test_macros.hpp>
#include "flockstorm/grid/get_cell.h"
#include "vectorstorm/vector/vector3.h"

TEST_CASE("get_cell computes correct grid cell", "[grid][get_cell]") {
  SECTION("origin with scale 1.0 returns zero cell") {
    auto const cell{flockstorm::grid::get_cell({0.0f, 0.0f, 0.0f}, 1.0f)};
    REQUIRE(cell.x == 0);
    REQUIRE(cell.y == 0);
    REQUIRE(cell.z == 0);
  }

  SECTION("positive coordinates are floored to cell index") {
    auto const cell{flockstorm::grid::get_cell({2.9f, 5.1f, 7.8f}, 1.0f)};
    REQUIRE(cell.x == 2);
    REQUIRE(cell.y == 5);
    REQUIRE(cell.z == 7);
  }

  SECTION("negative coordinates are floored correctly") {
    // floor(-1.5) == -2, floor(-2.1) == -3, floor(-0.1) == -1
    auto const cell{flockstorm::grid::get_cell({-1.5f, -2.1f, -0.1f}, 1.0f)};
    REQUIRE(cell.x == -2);
    REQUIRE(cell.y == -3);
    REQUIRE(cell.z == -1);
  }

  SECTION("scale 2.0 divides coordinates before flooring") {
    // floor(3.0/2.0)==1, floor(4.0/2.0)==2, floor(6.9/2.0)==3
    auto const cell{flockstorm::grid::get_cell({3.0f, 4.0f, 6.9f}, 2.0f)};
    REQUIRE(cell.x == 1);
    REQUIRE(cell.y == 2);
    REQUIRE(cell.z == 3);
  }

  SECTION("position exactly on a positive boundary belongs to that cell") {
    auto const cell{flockstorm::grid::get_cell({3.0f, 3.0f, 3.0f}, 1.0f)};
    REQUIRE(cell.x == 3);
    REQUIRE(cell.y == 3);
    REQUIRE(cell.z == 3);
  }

  SECTION("large scale groups distant positions into the same cell") {
    auto const cell0{flockstorm::grid::get_cell({0.5f, 0.5f, 0.5f}, 10.0f)};
    auto const cell1{flockstorm::grid::get_cell({9.9f, 9.9f, 9.9f}, 10.0f)};
    REQUIRE(cell0 == cell1);
  }

  SECTION("adjacent positions across a cell boundary end up in different cells") {
    auto const cell_below{flockstorm::grid::get_cell({0.99f, 0.0f, 0.0f}, 1.0f)};
    auto const cell_above{flockstorm::grid::get_cell({1.0f, 0.0f, 0.0f}, 1.0f)};
    REQUIRE(cell_below.x == 0);
    REQUIRE(cell_above.x == 1);
  }

  SECTION("default boid grid scale (collision avoidance range 3.5)") {
    // Positions within [0, 3.5) should map to cell 0
    auto const cell{flockstorm::grid::get_cell({3.49f, 0.0f, 0.0f}, 3.5f)};
    REQUIRE(cell.x == 0);
    // Position at exactly 3.5 maps to cell 1
    auto const cell_next{flockstorm::grid::get_cell({3.5f, 0.0f, 0.0f}, 3.5f)};
    REQUIRE(cell_next.x == 1);
  }
}
