#include "manager.h"
#include <iostream>
#include "vectorstorm/aabb/aabb3.h"
#include "memorystorm/memorystorm.h"

namespace flockstorm {

manager::manager(unsigned int this_num_boids)
  : num_boids(this_num_boids) {
  /// Default constructor
  std::cout << "FlockStorm: Initialising with " << this_num_boids << " boids." << std::endl;
  update_precomputed_quantities();
}

manager::~manager() {
  /// Default destructor
  std::cout << "FlockStorm: Shutting down." << std::endl;
}

void manager::update_precomputed_quantities() {
  /// Update the pre-computed values used by the simulation
  #ifdef DEBUG_FLOCKSTORM
    std::cout << "FlockStorm: DEBUG: Updating pre-computed values" << std::endl;
  #endif // DEBUG_FLOCKSTORM
  collision_avoidance_range_sq = collision_avoidance_range * collision_avoidance_range;
  velocity_matching_range_sq   = velocity_matching_range   * velocity_matching_range;
  flock_centering_range_sq     = flock_centering_range     * flock_centering_range;
  collision_avoidance_grid.scale          = collision_avoidance_range;
  collision_avoidance_obstacle_grid.scale = collision_avoidance_range;
  velocity_matching_grid.scale            = velocity_matching_range;
  flock_centering_grid.scale              = flock_centering_range;
  acceleration_max_sq          = acceleration_max          * acceleration_max;
  //speed_limit_max_sq           = speed_limit_max           * speed_limit_max;
  //speed_limit_min_sq           = speed_limit_min           * speed_limit_min;

  for(auto &it : obstacles.spheres) {
    it.update(collision_avoidance_range);
  }

  #ifdef DEBUG_FLOCKSTORM
    std::cout << "FlockStorm: DEBUG: === Updated simulation values === " << std::endl;
    std::cout << "FlockStorm: DEBUG: collision_avoidance_range " << collision_avoidance_range << std::endl;
    std::cout << "FlockStorm: DEBUG: collision_avoidance_scale " << collision_avoidance_scale << std::endl;
    std::cout << "FlockStorm: DEBUG: velocity_matching_range   " << velocity_matching_range << std::endl;
    std::cout << "FlockStorm: DEBUG: velocity_matching_scale   " << velocity_matching_scale << std::endl;
    std::cout << "FlockStorm: DEBUG: flock_centering_range     " << flock_centering_range << std::endl;
    std::cout << "FlockStorm: DEBUG: flock_centering_scale     " << flock_centering_scale << std::endl;
    std::cout << "FlockStorm: DEBUG: goal_seeking_scale        " << goal_seeking_scale << std::endl;
    std::cout << "FlockStorm: DEBUG: acceleration_max          " << acceleration_max << std::endl;
    std::cout << "FlockStorm: DEBUG: damping_factor            " << damping_factor << std::endl;
  #endif // DEBUG_FLOCKSTORM
};

size_t manager::add_obstacle_sphere(vec3f const &this_position, float this_radius) {
  /// Helper to add a sphere obstacle with precomputed values and return its index
  obstacles.spheres.emplace_back(this_position, this_radius, collision_avoidance_range);
  return obstacles.spheres.size() - 1;
}

void manager::distribute_boids_randomly(aabb3f const &bounding_box, std::mt19937::result_type seed) {
  /// Distribute the boids randomly within the specified bounding box
  #ifdef DEBUG_FLOCKSTORM
    std::cout << "FlockStorm: DEBUG: Distributing boids randomly through the range " << bounding_box << " with seed " << seed << std::endl;
  #endif // DEBUG_FLOCKSTORM
  std::mt19937 rng(seed);
  vec3<std::uniform_real_distribution<float>> pos_dist(std::uniform_real_distribution<float>{bounding_box.min.x, bounding_box.max.x},
                                                       std::uniform_real_distribution<float>{bounding_box.min.y, bounding_box.max.y},
                                                       std::uniform_real_distribution<float>{bounding_box.min.z, bounding_box.max.z});
  for(unsigned int i = 0; i != num_boids; ++i) {
    positions[i].assign(pos_dist.x(rng), pos_dist.y(rng), pos_dist.z(rng));
    velocities[i].assign();
    accelerations[i].assign();
  }
  populate_grids();
}
void manager::set_goal_position_randomly(aabb3f const &bounding_box, std::mt19937::result_type seed) {
  /// Position the goal randomly within the specified bounding box
  #ifdef DEBUG_FLOCKSTORM
    std::cout << "FlockStorm: DEBUG: Positioning goal randomly within the range " << bounding_box << " with seed " << seed << std::endl;
  #endif // DEBUG_FLOCKSTORM
  std::mt19937 rng(seed);
  vec3<std::uniform_real_distribution<float>> pos_dist(std::uniform_real_distribution<float>{bounding_box.min.x, bounding_box.max.x},
                                                       std::uniform_real_distribution<float>{bounding_box.min.y, bounding_box.max.y},
                                                       std::uniform_real_distribution<float>{bounding_box.min.z, bounding_box.max.z});
  goal_position.assign(pos_dist.x(rng), pos_dist.y(rng), pos_dist.z(rng));
}

vec3f const &manager::get_position(unsigned int boid_id) const {
  #ifdef NDEBUG
    return positions[boid_id];
  #else
    return positions.at(boid_id);                                               // safe check in debug mode only
  #endif // NDEBUG
}
vec3f const &manager::get_velocity(unsigned int boid_id) const {
  #ifdef NDEBUG
    return velocities[boid_id];
  #else
    return velocities.at(boid_id);                                              // safe check in debug mode only
  #endif // NDEBUG
}
vec3f const &manager::get_acceleration(unsigned int boid_id) const {
  #ifdef NDEBUG
    return accelerations[boid_id];
  #else
    return accelerations.at(boid_id);                                           // safe check in debug mode only
  #endif // NDEBUG
}

void manager::set_position(unsigned int boid_id, vec3f const &new_position) {
  #ifdef NDEBUG
    positions[boid_id] = new_position;
  #else
    positions.at(boid_id) = new_position;                                       // safe check in debug mode only
  #endif // NDEBUG
}
void manager::set_velocity(unsigned int boid_id, vec3f const &new_velocity) {
  #ifdef NDEBUG
    velocities[boid_id] = new_velocity;
  #else
    velocities.at(boid_id) = new_velocity;                                      // safe check in debug mode only
  #endif // NDEBUG
}
void manager::set_acceleration(unsigned int boid_id, vec3f const &new_acceleration) {
  #ifdef NDEBUG
    accelerations[boid_id] = new_acceleration;
  #else
    accelerations.at(boid_id) = new_acceleration;                               // safe check in debug mode only
  #endif // NDEBUG
}

std::vector<unsigned int> manager::get_grid_neighbour_boids(vec3i const &our_grid_cell,
                                                            grid::boid const &grid) {
  /// Return a list of boids in neighbouring grid cells in the specified grid
  std::vector<unsigned int> other_boids;
  // TODO: replace the above with boost's small vector
  for(int offset_x = -1; offset_x != 2; ++offset_x) {
    for(int offset_y = -1; offset_y != 2; ++offset_y) {
      for(int offset_z = -1; offset_z != 2; ++offset_z) {
        auto const &it(grid.grid.find(our_grid_cell + vec3i{offset_x, offset_y, offset_z}));
        if(it == grid.grid.end()) {
          continue;                                                             // nothing in this grid cell
        }
        auto &grid_cell(it->second);
        other_boids.insert(other_boids.end(), grid_cell.begin(), grid_cell.end());
      }
    }
  }
  return other_boids;
}

void manager::populate_grids() {
  /// Populate the grids
  #ifdef DEBUG_FLOCKSTORM
    std::cout << "FlockStorm: DEBUG: Updating grids" << std::endl;
  #endif // DEBUG_FLOCKSTORM

  collision_avoidance_obstacle_grid.clear();
  collision_avoidance_grid.clear();
  velocity_matching_grid.clear();
  flock_centering_grid.clear();

  // fixed obstacle grids
  for(unsigned int i = 0; i != obstacles.spheres.size(); ++i) {
    auto const &obstacle(obstacles.spheres[i]);
    float const bounding_box_radius = obstacle.radius + collision_avoidance_obstacle_grid.scale;
    aabb3f const obstacle_bounds(obstacle.position - bounding_box_radius,
                                 obstacle.position + bounding_box_radius);
    aabb3i const obstacle_bounds_grid(collision_avoidance_obstacle_grid.get_cell(obstacle_bounds.min),
                                      collision_avoidance_obstacle_grid.get_cell(obstacle_bounds.max));
    vec3i cell;
    float bounding_box_radius_padded_sq = bounding_box_radius + (collision_avoidance_obstacle_grid.scale * 0.5f);
    bounding_box_radius_padded_sq *= bounding_box_radius_padded_sq;
    #ifdef DEBUG_FLOCKSTORM
      std::cout << "FlockStorm: DEBUG: obstacle " << i << " bounding box " << obstacle_bounds << std::endl;
    #endif // DEBUG_FLOCKSTORM
    for(cell.x = obstacle_bounds_grid.min.x; cell.x != obstacle_bounds_grid.max.x; ++cell.x) {
      for(cell.y = obstacle_bounds_grid.min.y; cell.y != obstacle_bounds_grid.max.y; ++cell.y) {
        for(cell.z = obstacle_bounds_grid.min.z; cell.z != obstacle_bounds_grid.max.z; ++cell.z) {
          vec3f const cell_centre_coords((vec3f(cell) + 0.5f) * collision_avoidance_obstacle_grid.scale);
          float const dist_sq = (cell_centre_coords - obstacle.position).length_sq();
          if(dist_sq <= bounding_box_radius_padded_sq) {
            collision_avoidance_obstacle_grid.grid[cell].emplace_back(i);
            #ifdef DEBUG_FLOCKSTORM
              std::cout << "FlockStorm: DEBUG: obstacle " << i << " is included in cell " << cell << std::endl;
            #endif // DEBUG_FLOCKSTORM
          }
        }
      }
    }
  }

  // boid grids
  for(unsigned int i = 0; i != num_boids; ++i) {
    {
      vec3i const collision_avoidance_grid_cell(collision_avoidance_grid.get_cell(positions[i]));
      collision_avoidance_grid.occupied_cells[i] = collision_avoidance_grid_cell;
      collision_avoidance_grid.grid[collision_avoidance_grid_cell].emplace_back(i);
    }
    {
      vec3i const velocity_matching_grid_cell(velocity_matching_grid.get_cell(positions[i]));
      velocity_matching_grid.occupied_cells[i] = velocity_matching_grid_cell;
      velocity_matching_grid.grid[velocity_matching_grid_cell].emplace_back(i);
    }
    {
      vec3i const flock_centering_grid_cell(flock_centering_grid.get_cell(positions[i]));
      flock_centering_grid.occupied_cells[i] = flock_centering_grid_cell;
      flock_centering_grid.grid[flock_centering_grid_cell].emplace_back(i);
    }
  }

  #ifndef NDEBUG
    dump_grid_memory_usage();
  #endif // NDEBUG
}

void manager::dump_grid_memory_usage() {
  /// Output statistics about grid memory usage
  size_t collision_avoidance_grid_size          = sizeof(collision_avoidance_grid);
  size_t collision_avoidance_obstacle_grid_size = sizeof(collision_avoidance_obstacle_grid);
  size_t velocity_matching_grid_size            = sizeof(velocity_matching_grid);
  size_t flock_centering_grid_size              = sizeof(flock_centering_grid);
  for(auto const &it : collision_avoidance_grid.grid) {
    collision_avoidance_grid_size += it.second.size() * sizeof(unsigned int);
  }
  for(auto const &it : collision_avoidance_obstacle_grid.grid) {
    collision_avoidance_obstacle_grid_size += it.second.size() * sizeof(unsigned int);
  }
  for(auto const &it : velocity_matching_grid.grid) {
    velocity_matching_grid_size += it.second.size() * sizeof(unsigned int);
  }
  for(auto const &it : flock_centering_grid.grid) {
    flock_centering_grid_size += it.second.size() * sizeof(unsigned int);
  }
  std::cout << "FlockStorm: Memory usage: Collision avoidance (boid):     " << collision_avoidance_grid.grid.size() << " elements, size " << memorystorm::human_readable(collision_avoidance_grid_size) << std::endl;
  std::cout << "FlockStorm: Memory usage: Collision avoidance (obstacle): " << collision_avoidance_obstacle_grid.grid.size() << " elements, size " << memorystorm::human_readable(collision_avoidance_obstacle_grid_size) << std::endl;
  std::cout << "FlockStorm: Memory usage: Velocity matching:              " << velocity_matching_grid.grid.size() << " elements, size " << memorystorm::human_readable(velocity_matching_grid_size) << std::endl;
  std::cout << "FlockStorm: Memory usage: Flock centering:                " << flock_centering_grid.grid.size() << " elements, size " << memorystorm::human_readable(flock_centering_grid_size) << std::endl;
}

void manager::update() {
  /// Update boids
  for(unsigned int i = 0; i != num_boids; ++i) {
    // calculate accelerations on this boid this tick
    auto &position     = positions[i];
    //auto &velocity     = velocities[i];
    auto &acceleration = accelerations[i];
    acceleration.assign();

    {
      // calculate collision avoidance acceleration from external obstacles
      #ifndef NDEBUG
        if(!obstacles.spheres.empty()) {
          if(collision_avoidance_obstacle_grid.grid.empty()) {
            std::cout << "FlockStorm: ERROR: Obstacles exist, but obstacle grid is unpopulated - don't forget to run populate_grids()!" << std::endl;
          }
        }
      #endif // NDEBUG
      vec3i const collision_avoidance_grid_cell(collision_avoidance_obstacle_grid.get_cell(positions[i]));
      auto const &it(collision_avoidance_obstacle_grid.grid.find(collision_avoidance_grid_cell));
      if(it != collision_avoidance_obstacle_grid.grid.end()) {
        for(auto const &obstacle_id : it->second) {
          auto const &obstacle(obstacles.spheres[obstacle_id]);
          float const dist_centre_sq = (obstacle.position - position).length_sq();
          if(dist_centre_sq < obstacle.collision_avoidance_range_sq) {
            float dist_sq = (obstacle.position - position).length() - obstacle.radius;
            dist_sq *= dist_sq;
            float const acc = collision_avoidance_scale / dist_sq;
            acceleration += (obstacle.position - position).normalise_copy() * -acc; // division by zero possible - assuming we never overlap the centre of an obstacle
          }
        }
      }
    }
    if(acceleration.length_sq() > acceleration_max_sq) {                        // clamp acceleration to the maximum
      #ifdef DEBUG_FLOCKSTORM
        std::cout << "FlockStorm: DEBUG: Boid " << i << ": Early exit at obstacle collision avoidance with acceleration " << acceleration << std::endl;
      #endif // DEBUG_FLOCKSTORM
      acceleration.normalise();
      acceleration *= acceleration_max_sq;
      continue;                                                                 // we can't accumulate any more accelerations here, so early exit
    }

    {
      // calculate collision avoidance acceleration from other boids
      vec3i const collision_avoidance_grid_cell(collision_avoidance_grid.get_cell(positions[i]));
      // TODO: combine the above with the grid update below
      for(unsigned int j : get_grid_neighbour_boids(collision_avoidance_grid_cell, collision_avoidance_grid)) {
        if(j == i) {
          continue;
        }
        float const dist_sq = (positions[j] - position).length_sq();
        if(dist_sq < collision_avoidance_range_sq) {
          float const acc = collision_avoidance_scale / dist_sq;
          acceleration += (positions[j] - position).normalise_copy() * -acc;    // division by zero possible - assuming we never overlap another boid
        }
      }
    }
    if(acceleration.length_sq() > acceleration_max_sq) {                        // clamp acceleration to the maximum
      #ifdef DEBUG_FLOCKSTORM
        std::cout << "FlockStorm: DEBUG: Boid " << i << ": Early exit at boid collision avoidance with acceleration " << acceleration << std::endl;
      #endif // DEBUG_FLOCKSTORM
      acceleration.normalise();
      acceleration *= acceleration_max_sq;
      continue;                                                                 // we can't accumulate any more accelerations here, so early exit
    }

    {
      // calculate velocity matching acceleration
      vec3f flock_velocity;
      unsigned int flock_count = 0;
      vec3i const velocity_matching_grid_cell(velocity_matching_grid.get_cell(positions[i]));
      // TODO: combine the above with the grid update below
      for(unsigned int j : get_grid_neighbour_boids(velocity_matching_grid_cell, velocity_matching_grid)) {
        if(j == i) {
          continue;
        }
        float const dist_sq = (positions[j] - position).length_sq();
        if(dist_sq < velocity_matching_range_sq) {
          float const acc = 1.0f / dist_sq;
          flock_velocity += velocities[j] + acc;
          ++flock_count;
        }
      }
      if(flock_count != 0) {
        flock_velocity /= static_cast<float>(flock_count);
        acceleration += flock_velocity * velocity_matching_scale;
      }
    }
    if(acceleration.length_sq() > acceleration_max_sq) {                        // clamp acceleration to the maximum
      #ifdef DEBUG_FLOCKSTORM
        std::cout << "FlockStorm: DEBUG: Boid " << i << ": Early exit at velocity matching with acceleration " << acceleration << std::endl;
      #endif // DEBUG_FLOCKSTORM
      acceleration.normalise();
      acceleration *= acceleration_max_sq;
      continue;                                                                 // we can't accumulate any more accelerations here, so early exit
    }

    {
      // calculate flock centering acceleration
      vec3f flock_centroid;
      unsigned int flock_count = 0;
      vec3i const flock_centering_grid_cell(flock_centering_grid.get_cell(positions[i]));
      // TODO: combine the above with the grid update below
      for(unsigned int j : get_grid_neighbour_boids(flock_centering_grid_cell, flock_centering_grid)) {
        if(j == i) {
          continue;
        }
        float const dist_sq = (positions[j] - position).length_sq();
        if(dist_sq < flock_centering_range_sq) {
          flock_centroid += positions[j];
          ++flock_count;
        }
      }
      if(flock_count != 0) {
        flock_centroid /= static_cast<float>(flock_count);
        vec3f const offset_from_flock_centroid(flock_centroid - position);
        float const acc = flock_centering_scale / offset_from_flock_centroid.length_sq();
        acceleration += offset_from_flock_centroid.normalise_safe_copy() * acc;
      }
    }
    if(acceleration.length_sq() > acceleration_max_sq) {                        // clamp acceleration to the maximum
      #ifdef DEBUG_FLOCKSTORM
        std::cout << "FlockStorm: DEBUG: Boid " << i << ": Early exit at flock centering with acceleration " << acceleration << std::endl;
      #endif // DEBUG_FLOCKSTORM
      acceleration.normalise();
      acceleration *= acceleration_max_sq;
      continue;                                                                 // we can't accumulate any more accelerations here, so early exit
    }

    {
      // calculate goal-seeking acceleration
      vec3f const offset_from_goal(goal_position - position);
      acceleration += offset_from_goal.normalise_safe_copy() * goal_seeking_scale;
      //acceleration += offset_from_goal * goal_seeking_scale;
    }
    if(acceleration.length_sq() > acceleration_max_sq) {                        // clamp acceleration to the maximum
      #ifdef DEBUG_FLOCKSTORM
        std::cout << "FlockStorm: DEBUG: Boid " << i << ": Early exit at goal seeking with acceleration " << acceleration << std::endl;
      #endif // DEBUG_FLOCKSTORM
      acceleration.normalise();
      acceleration *= acceleration_max_sq;
      continue;                                                                 // we can't accumulate any more accelerations here, so early exit
    }
  }

  // apply motion
  for(unsigned int i = 0; i != num_boids; ++i) {
    // TODO: simd-ify this
    // update velocities
    velocities[i] += accelerations[i];
    // apply damping
    velocities[i] *= damping_factor;
    positions[i] += velocities[i];
    #ifdef DEBUG_FLOCKSTORM
      std::cout << "FlockStorm: DEBUG: Boid " << i << ": final acceleration " << accelerations[i] << std::endl;
      std::cout << "FlockStorm: DEBUG: Boid " << i << ": final velocity     " << velocities[i] << std::endl;
      std::cout << "FlockStorm: DEBUG: Boid " << i << ": final position     " << positions[i] << std::endl;
    #endif // DEBUG_FLOCKSTORM
  }

  // update grid cells if necessary
  for(unsigned int i = 0; i != num_boids; ++i) {
    collision_avoidance_grid.update(i, positions[i]);
    velocity_matching_grid.update(  i, positions[i]);
    flock_centering_grid.update(    i, positions[i]);
  }
}

}
