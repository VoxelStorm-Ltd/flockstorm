#pragma once

#include <random>
#include "vectorstorm/aabb/aabb3_forward.h"
#include "obstacle/sphere.h"
#include "grid/boid.h"

namespace flockstorm {

class manager {
public:
  // global simulation parameters
  unsigned int const num_boids{1000};

  float collision_avoidance_range{3.5f};                                        // default values are optimised for ticks at 10Hz
  float collision_avoidance_scale{0.10f};
  float velocity_matching_range{  5.0f};
  float velocity_matching_scale{  0.05f};
  float flock_centering_range{    7.0f};
  float flock_centering_scale{    0.06f};
  float goal_seeking_scale{       0.02f};
  float acceleration_max{         0.30f};
  float damping_factor{           0.953f};
  //float speed_limit_max{          1.00f};
  //float speed_limit_min{          0.01f};

  vec3f goal_position;                                                          // the overall flock location goal
  struct {
    std::vector<obstacle::sphere> spheres;                                      // obstacle container - spheres

    void clear() {
      spheres.clear();
    }
  } obstacles;

private:
  // pre-computed quantities
  float collision_avoidance_range_sq{0.0f};                                     // automatically updated
  float velocity_matching_range_sq{  0.0f};
  float flock_centering_range_sq{    0.0f};
  float acceleration_max_sq{         0.0f};
  //float speed_limit_max_sq{          0.0f};
  //float speed_limit_min_sq{          0.0f};

  // individual boid properties - use struct-of-arrays layout for speed
  #ifdef FLOCKSTORM_USE_STACK
    std::array<vec3f, num_boids> positions;
    std::array<vec3f, num_boids> velocities;
    std::array<vec3f, num_boids> accelerations;
    std::array<unsigned int, num_boids> grid_neighbour_boids;
  #else
    std::vector<vec3f> positions{num_boids};
    std::vector<vec3f> velocities{num_boids};
    std::vector<vec3f> accelerations{num_boids};
    std::vector<unsigned int> grid_neighbour_boids{num_boids};
  #endif // FLOCKSTORM_USE_STACK

  // space-dividing grids for influencers of the various boid forces
  grid::base collision_avoidance_obstacle_grid;
  grid::boid collision_avoidance_grid{num_boids};
  grid::boid velocity_matching_grid{num_boids};
  grid::boid flock_centering_grid{num_boids};

public:
  manager(unsigned int this_num_boids);
  ~manager();

  void update_precomputed_quantities();

  size_t add_obstacle_sphere(vec3f const &this_position, float this_radius);
  void distribute_boids_randomly(aabb3f const &bounding_box, std::mt19937::result_type seed = 0);
  void set_goal_position_randomly(aabb3f const &bounding_box, std::mt19937::result_type seed = 0);

  #ifdef NDEBUG
    vec3f const &get_position(    unsigned int boid_id) const __attribute__((__pure__));
    vec3f const &get_velocity(    unsigned int boid_id) const __attribute__((__pure__));
    vec3f const &get_acceleration(unsigned int boid_id) const __attribute__((__pure__));
  #else
    vec3f const &get_position(    unsigned int boid_id) const;
    vec3f const &get_velocity(    unsigned int boid_id) const;
    vec3f const &get_acceleration(unsigned int boid_id) const;
  #endif // NDEBUG

  void set_position(    unsigned int boid_id, vec3f const &new_position);
  void set_velocity(    unsigned int boid_id, vec3f const &new_velocity);
  void set_acceleration(unsigned int boid_id, vec3f const &new_acceleration);

  void update_grid_neighbour_boids(vec3i const &our_grid_square, grid::boid const &grid);
  void populate_grids();
  void dump_grid_memory_usage();

  void update();

  void update_partial(unsigned int begin, unsigned int end);
  void update_partial_finalise();
};

}
