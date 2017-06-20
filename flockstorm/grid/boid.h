#ifndef FLOCKSTORM_GRID_BOID_INCLUDED
#define FLOCKSTORM_GRID_BOID_INCLUDED

#include "base.h"

namespace flockstorm::grid {

struct boid : base {
  unsigned int const num_boids = 1000;

  #ifdef FLOCKSTORM_USE_STACK
    std::array<vec3i, num_boids> occupied_cells;
  #else
    std::vector<vec3i> occupied_cells{num_boids};
  #endif // FLOCKSTORM_USE_STACK

  boid(unsigned int this_num_boids);

  void update(unsigned int boid_id, vec3f const &position);
};

}

#endif // FLOCKSTORM_GRID_BOID_INCLUDED
