#include "boid.h"
#ifndef NDEBUG
  #include <iostream>
#endif // NDEBUG
#ifdef VECTORSTORM_NO_BOOST
  #include <algorithm>
#endif // VECTORSTORM_NO_BOOST

namespace flockstorm::grid {

boid::boid(unsigned int this_num_boids)
  : num_boids(this_num_boids) {
  /// Default constructor specifying number of boids
}

void boid::update(unsigned int boid_id, vec3f const &position) {
  /// Update this grid for this boid's new position
  vec3i const grid_cell(get_cell(position));
  if(occupied_cells[boid_id] != grid_cell) {
    // remove the boid id from the last cell it was in
    auto const &old_grid_cell_it(grid.find(occupied_cells[boid_id]));
    #ifndef NDEBUG
      if(old_grid_cell_it == grid.end()) {                                      // safety check
        std::cout << "ERROR: Boid " << boid_id << " had empty old grid cell - this should never happen!" << std::endl;
      }
    #endif // NDEBUG
    auto &old_grid_cell(old_grid_cell_it->second);
    if(old_grid_cell.size() == 1) {
      // this is the only entry in the cell, so remove the entire cell from the map and free the memory
      grid.erase(old_grid_cell_it);
    } else {
      // optimised vector removal when we don't care about order
      auto it = std::find(old_grid_cell.begin(), old_grid_cell.end(), boid_id);
      #ifndef NDEBUG
        if(it == old_grid_cell.end()) {                                         // safety check
          std::cout << "ERROR: Did not find boid " << boid_id << " in old grid cell - this should never happen!" << std::endl;
        } else {
      #endif // NDEBUG
      *it = old_grid_cell.back();                                               // replace this boid id in the cell with the last boid id in the cell, since we don't care about order
      old_grid_cell.resize(old_grid_cell.size() - 1);                           // then shrink it to bump off the duplicated last element - we don't check for size != 0 here because there must be at least one element to remove
      #ifndef NDEBUG
        }
      #endif // NDEBUG
    }

    // add the boid id to the new cell
    occupied_cells[boid_id] = grid_cell;
    grid[grid_cell].emplace_back(boid_id);
  }
}

}
