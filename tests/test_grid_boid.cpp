#include <catch2/catch_test_macros.hpp>
#include <algorithm>
#include "flockstorm/grid/boid.h"

// Helper: manually populate a boid grid as populate_grids() would
static void populate_single_boid(flockstorm::grid::boid &g, unsigned int const boid_id, vec3f const &pos) {
  vec3i const cell{g.get_cell(pos)};
  g.occupied_cells[boid_id] = cell;
  g.grid[cell].emplace_back(boid_id);
}

TEST_CASE("grid::boid - construction", "[grid][boid]") {
  SECTION("stores num_boids correctly") {
    flockstorm::grid::boid g{10};
    REQUIRE(g.num_boids == 10);
  }

  SECTION("occupied_cells vector has correct size") {
    flockstorm::grid::boid g{5};
    REQUIRE(g.occupied_cells.size() == 5);
  }

  SECTION("grid starts empty after construction") {
    flockstorm::grid::boid g{3};
    REQUIRE(g.grid.empty());
  }
}

TEST_CASE("grid::boid - update is a no-op when the boid stays in the same cell", "[grid][boid]") {
  flockstorm::grid::boid g{2};
  g.scale = 1.0f;

  vec3f pos0{0.5f, 0.5f, 0.5f};
  vec3f pos1{5.0f, 5.0f, 5.0f};
  populate_single_boid(g, 0, pos0);
  populate_single_boid(g, 1, pos1);

  size_t const initial_cells{g.grid.size()};

  // Move boid 0 within the same cell {0,0,0}
  g.update(0, {0.9f, 0.1f, 0.2f});

  REQUIRE(g.grid.size() == initial_cells);
  REQUIRE(g.occupied_cells[0] == vec3i{0, 0, 0});
}

TEST_CASE("grid::boid - update moves boid to a new cell", "[grid][boid]") {
  flockstorm::grid::boid g{1};
  g.scale = 1.0f;

  vec3f initial_pos{0.5f, 0.5f, 0.5f};
  populate_single_boid(g, 0, initial_pos);

  vec3f new_pos{3.5f, 3.5f, 3.5f};  // cell {3,3,3}
  g.update(0, new_pos);

  // Occupied cell record should be updated
  REQUIRE(g.occupied_cells[0] == vec3i{3, 3, 3});

  // Old cell {0,0,0} should have been erased (it was the only occupant)
  REQUIRE(g.grid.find({0, 0, 0}) == g.grid.end());

  // New cell {3,3,3} should contain boid 0
  auto it{g.grid.find({3, 3, 3})};
  REQUIRE(it != g.grid.end());
  REQUIRE(std::find(it->second.begin(), it->second.end(), 0u) != it->second.end());
}

TEST_CASE("grid::boid - update removes boid from a cell shared with other boids", "[grid][boid]") {
  flockstorm::grid::boid g{2};
  g.scale = 1.0f;

  // Both boids start in cell {0,0,0}
  vec3i const cell0{0, 0, 0};
  g.occupied_cells[0] = cell0;
  g.occupied_cells[1] = cell0;
  g.grid[cell0] = {0, 1};

  // Move boid 0 out of cell {0,0,0}
  g.update(0, {5.0f, 5.0f, 5.0f});  // cell {5,5,5}

  // Cell {0,0,0} should still exist and contain only boid 1
  auto it{g.grid.find(cell0)};
  REQUIRE(it != g.grid.end());
  REQUIRE(it->second.size() == 1);
  REQUIRE(it->second[0] == 1u);

  // Boid 0 should be in its new cell
  REQUIRE(g.occupied_cells[0] == vec3i{5, 5, 5});
  auto it2{g.grid.find({5, 5, 5})};
  REQUIRE(it2 != g.grid.end());
  REQUIRE(std::find(it2->second.begin(), it2->second.end(), 0u) != it2->second.end());
}

TEST_CASE("grid::boid - multiple sequential updates track the correct cell", "[grid][boid]") {
  flockstorm::grid::boid g{1};
  g.scale = 2.0f;

  populate_single_boid(g, 0, {0.0f, 0.0f, 0.0f});
  REQUIRE(g.occupied_cells[0] == vec3i{0, 0, 0});

  // Move to cell {1,0,0} (scale=2 means x in [2,4) -> cell 1)
  g.update(0, {2.5f, 0.0f, 0.0f});
  REQUIRE(g.occupied_cells[0] == vec3i{1, 0, 0});

  // Move to cell {2,0,0} (x in [4,6) -> cell 2)
  g.update(0, {5.0f, 0.0f, 0.0f});
  REQUIRE(g.occupied_cells[0] == vec3i{2, 0, 0});

  // Only one cell should be occupied at any time
  REQUIRE(g.grid.size() == 1);
}
