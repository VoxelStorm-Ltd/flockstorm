#pragma once

#include <vector>
#include <unordered_map>
#include <boost/functional/hash.hpp>
#include "vectorstorm/vector/vector3.h"

namespace flockstorm::grid {

struct base {
  using cell = std::vector<unsigned int>;
  std::unordered_map<vec3i, cell> grid;
  //boost::container::flat_map<vec3i, cell> grid;
  float scale = 1.0f;
  ///float scale_inv = 1.0f;

  void clear();

  vec3i get_cell(vec3f const &position) const;
};

}
