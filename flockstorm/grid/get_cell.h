#pragma once

#include "vectorstorm/vector/vector3_forward.h"

namespace flockstorm::grid {

vec3i get_cell(vec3f const &position, float grid_scale);

}
