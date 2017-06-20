#include "get_cell.h"
#include "vectorstorm/vector/vector3.h"
#include "vectorstorm/floor_fast.h"

namespace flockstorm::grid {

vec3i get_cell(vec3f const &position, float grid_scale) {
  /// Return a 3D grid cell, given a 3D position and a grid scale
  return {
    floor_fast(position.x / grid_scale),
    floor_fast(position.y / grid_scale),
    floor_fast(position.z / grid_scale)
  };
}

}
