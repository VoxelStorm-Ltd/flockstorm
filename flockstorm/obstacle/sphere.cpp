#include "sphere.h"
#ifdef DEBUG_FLOCKSTORM
  #include <iostream>
#endif // DEBUG_FLOCKSTORM

namespace flockstorm::obstacle {

sphere::sphere(vec3f const &this_position, float this_radius, float boid_collision_avoidance_range)
  : position(this_position),
    radius(this_radius) {
  /// Default constructor
  update(boid_collision_avoidance_range);
  #ifdef DEBUG_FLOCKSTORM
    std::cout << "FlockStorm: DEBUG: Created a sphere obstacle at " << position << " radius " << radius << std::endl;
  #endif // DEBUG_FLOCKSTORM
}

void sphere::update(float boid_collision_avoidance_range) {
  collision_avoidance_range_sq = (radius + boid_collision_avoidance_range);
  collision_avoidance_range_sq *= collision_avoidance_range_sq;
}

}
