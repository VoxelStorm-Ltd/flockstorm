# FlockStorm
C++ "[Boids](https://en.wikipedia.org/wiki/Boids)" high performance flocking library by VoxelStorm, as used in the background of the game [sphereFACE](http://sphereface.com).

The basic intent is to produce a visually believable real-time flocking simulation for large numbers of entities, with minimal performance cost.  This specific design is intended to be suitable for use in games and other real-time graphics applications, where the flocking simulation is not the primary purpose of the software - a simulation that can run below the game's target framerate, and the game engine can interpolate between simulation ticks.

To understand the original idea behind the "Boids" flocking simulation, refer to the paper [Reynolds, Craig (1987). Flocks, herds and schools: A distributed behavioral model. SIGGRAPH '87: Proceedings of the 14th Annual Conference on Computer Graphics and Interactive Techniques.](http://www.red3d.com/cwr/papers/1987/boids.html).

This documentation is limited in scope - as this is a component of a legacy game, it is released as open source without promise of ongoing support, but in the hope it will be useful.  Pull requests for extensions and optimisations are welcome.

## Dependencies
- Boost functional hash headers `<boost/functional/hash.hpp>`
- [VectorStorm](https://github.com/VoxelStorm-Ltd/vectorstorm)
- [MemoryStorm](https://github.com/VoxelStorm-Ltd/memorystorm)

## Design

This library uses a 3D grid, and tracks boid occupation within this grid, to minimise the number of interactions each frame.  It tries to be as light on CPU usage as possible, to allow the boids simulation to be used seamlessly within games for features such as visual bird flocks or schools of fish, without interfering with the primary CPU time budget.

In effect it is a very simple entity component system (ECS), with one fixed component type.

### Implementation best practices

The default values are optimised to give realistic and aesthetically pleasing results for infrequent ticks - 10Hz by default.  The intention is that your game engine smoothly interpolates each visual boid position between simulation ticks, keeping CPU usage to a minimum.  Run each tick after a period of real-clock time has elapsed, for example 100ms.  After each simulation tick, collect all boid positions, and for each render frame, interpolate towards this new state from the previous state.

Rather than using each boid to represent a single visual member of the flock, it's common practice to use each member to represent a pre-defined animated group of entities, for example a group of five or so birds flying together.  This allows for much larger flocks with a minimal increase in resource usage.

It is relatively expensive to update the simulation parameters, so this should not be done every tick.  However, this can be used to make dramatic effects - for example increasing `collision_avoidance_scale` to a large value suddenly can make the boids scatter, for an effect such as birds flying from treetops when they hear a gunshot.  Refer to public variables of `manager` below.

Updating `goal_position` is very cheap, and can be done every frame without added cost - use this to guide the flock, and lead them to a specific target.

## Example

A basic example of usage (details ommitted) - 

```cpp
#include <flockstorm/flockstorm.h>

...

// setup:
unsigned int constexpr num_boids{100};
flockstorm::manager boids{num_boids}; // create simulation with 100 boids
boids.add_obstacle_sphere(this_sphere.position, this_sphere.radius); // add a sphere-shaped obstacle to avoid
std::vector<vec3f> boid_positions_last{num_boids};
std::vector<vec3f> boid_positions_next{num_boids};
...

// in main loop:

std::vector<vec3f> boid_positions_current{num_boids};

if(time_to_update) {
  // if it's time to update, update the simulation
  boids.goal_position = next_target_position; // optionally update the target position
  
  boids.update(); // update the simulation

  std::swap(boid_positions_last, boid_positions_next); // ping-pong between our position containers to avoid copying
  for(unsigned int i = 0; i != boids.num_boids; ++i) {
    boid_positions_next[i] = boids.get_position(i) * world_scale;
    boid_positions_current[i] = boid_positions_last[i];
  }
} else {
  // if it's not time to update, interpolate the positions for this render frame:
  for(unsigned int i = 0; i != boids.num_boids; ++i) {
    float const factor = frames_until_update / update_delay;
    boid_positions_current[i] = boid_positions_next[i].lerp(factor, boid_positions_last[i]);
  }
}

// then draw your boids at boid_positions_current
```

## Concepts

### Boids

A "boid" is a single member of the flock.  They are modelled as a struct-of-arrays (SOA) and you would normally never need to access boids individually - instead, use the `manager` class described below.

```cpp
  boid(unsigned int this_num_boids);
  void update(unsigned int boid_id, vec3f const &position);
```
  
### Obstacles
```cpp
namespace flockstorm::obstacle { ... }
```
At present, only sphere obstacles are supported (as this is all that was required in [sphereFACE](http://sphereface.com)).  Others can be extended here easily.  Pull requests for other obstacle types are welcome - it could be extended easily to support cylinders, cubes, arbitrary meshes, etc.

Note that obstacles do not need to be created manually - it is easier to use the helper functions in `manager` (see below).

#### Sphere

```cpp
flockstorm::obstacle::sphere
```

A spherical obstacle has a 3D position and radius.

```cpp
vec3f position;
float radius = 1.0f;
float collision_avoidance_range_sq = 1.0f;                                    // automatically updated
```

Boids will try to avoid the obstacle by a minimum collision avoidance range - this can be zero, which will allow boids to brush against the radius, or it can be larger, to realistically have them steer clearer of obstacles.  To set the collision avoidance range, use:

```cpp
void update(float boid_collision_avoidance_range);
```

Also call this function after modifying the radius.  `collision_avoidance_range_sq` is updated automatically, for fast distance comparison computations avoiding square root.

### Grid

It's possible to select whether the list of occupied grid cells is stored on the heap or on the stack - see the `FLOCKSTORM_USE_STACK` define described below.  Bounded stack usage is possible because the maximum number of occupied cells is known in advance to be equal to the number of boids - at most, if every boid is in a different cell, there are that many occupied cells.  There can never be more, as boids can never occupy more than one cell at a time.

```cpp
namespace flockstorm::grid {
  void clear();
  vec3i get_cell(vec3f const &position) const;
}
```

### Manager

To easily manage the boid swarm, construct a manager instance and specify the number of boids:
```cpp
  flockstorm::manager manager(unsigned int num_boids);
```

The manager has a number of public variables which define the primary quantities of the simulation - these can be accessed directly:
```cpp
  float collision_avoidance_range = 3.5f;                                       // default values are optimised for ticks at 10Hz
  float collision_avoidance_scale = 0.10f;
  float velocity_matching_range   = 5.0f;
  float velocity_matching_scale   = 0.05f;
  float flock_centering_range     = 7.0f;
  float flock_centering_scale     = 0.06f;
  float goal_seeking_scale        = 0.02f;
  float acceleration_max          = 0.30f;
  float damping_factor            = 0.953f;
```

After modifying primary quantities, call 
```cpp
manager.update_precomputed_quantities();
```

The flock goal position can be modified at any time - there's no need to call an update function:
```cpp
manager.goal_position.assign(1.0f, 2.0f, 3.0f); // the overall flock location goal
```

The obstacles (currently only spheres) can be accessed directly through the member struct `obstacles`:
```cpp
struct {
  std::vector<obstacle::sphere> spheres;                                      // obstacle container - spheres
  void clear();
} obstacles;
```

Manager has the following public member functions:
```cpp
  size_t add_obstacle_sphere(vec3f const &this_position, float this_radius);
  void distribute_boids_randomly(aabb3f const &bounding_box, std::mt19937::result_type seed = 0);
  void set_goal_position_randomly(aabb3f const &bounding_box, std::mt19937::result_type seed = 0);

  vec3f const &get_position(    unsigned int boid_id) const;
  vec3f const &get_velocity(    unsigned int boid_id) const;
  vec3f const &get_acceleration(unsigned int boid_id) const;

  void set_position(    unsigned int boid_id, vec3f const &new_position);
  void set_velocity(    unsigned int boid_id, vec3f const &new_velocity);
  void set_acceleration(unsigned int boid_id, vec3f const &new_acceleration);

  std::vector<unsigned int> get_grid_neighbour_boids(vec3i const &our_grid_square, grid::boid const &grid);
  void populate_grids();
  void dump_grid_memory_usage();

  void update();
```

To actually run the simulation, call `update()` each simulation frame (as mentioned above, this is ideally not every render frame).


## Configuration

The following compile-time defines are available:

- `DEBUG_FLOCKSTORM` - Extra debugging output (dumps computed values to console when `update_precomputed_quantities()` is called).
- `FLOCKSTORM_USE_STACK` - Allocate all containers on the stack rather than the heap.
