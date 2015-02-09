#ifndef KERNEL_H
#define KERNEL_H



#include <stdint.h>
#include "utils/intpoint.h"
#include "utils/floatpoint.h"

typedef int32_t spaceType; // primitive type of the coordinates in 3D space
// could be easily replaced if we want to have more precision

typedef int64_t spaceTypeD; // double the primitive type of the coordinates in 3D space

typedef Point3 Point;

typedef Point3d PointD; // double precision / double the size

typedef FPoint3 FPoint;

#endif // KERNEL_H
