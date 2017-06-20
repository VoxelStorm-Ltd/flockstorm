#ifndef FLOCKSTORM_OBSTACLE_SPHERE_H_INCLUDED
#define FLOCKSTORM_OBSTACLE_SPHERE_H_INCLUDED

#include "vectorstorm/vector/vector3.h"

namespace flockstorm::obstacle {

struct sphere {
  vec3f position;
  float radius = 1.0f;
  float collision_avoidance_range_sq = 1.0f;                                    // automatically updated

  sphere(vec3f const &this_position, float this_radius, float boid_collision_avoidance_range);

  void update(float boid_collision_avoidance_range);
};

}

#endif // FLOCKSTORM_OBSTACLE_SPHERE_H_INCLUDED
