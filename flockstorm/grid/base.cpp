#include "base.h"
#include "get_cell.h"

namespace flockstorm::grid {

void base::clear() {
  /// Clear this grid
  grid.clear();
}

vec3i base::get_cell(vec3f const &position) const {
  /// Helper to get a cell by position
  return grid::get_cell(position, scale);
}

}
