#ifndef KERNEL_H
#define KERNEL_H


#define DEBUG_HERE std::cerr << __FILE__ << " : " << __LINE__ << std::endl

#include <stdint.h>
#include "utils/intpoint.h"

typedef int32_t spaceType; // primitive type of the coordinates in 3D space
// could be easily replaced if we want to have more precision

typedef int64_t spaceTypeD; // double the primitive type of the coordinates in 3D space

typedef Point3 Point;

typedef Point3d PointD; // double precision / double the size

static const spaceType MELD_DISTANCE = 30;

#endif // KERNEL_H
