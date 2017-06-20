#ifndef FLOCKSTORM_GRID_GET_CELL_H_INCLUDED
#define FLOCKSTORM_GRID_GET_CELL_H_INCLUDED

#include "vectorstorm/vector/vector3_forward.h"

namespace flockstorm::grid {

vec3i get_cell(vec3f const &position, float grid_scale);

}

#endif // GET_CELL_H_INCLUDED
