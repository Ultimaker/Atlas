#ifndef BOUNDING_BOX_H
#define BOUNDING_BOX_H

#include <algorithm> // min, max
#include "utils/intpoint.h" // Point3

#include "Kernel.h"

// enable/disable debug output
#define BBOX_DEBUG 0

#if BBOX_DEBUG == 1
#   define BBOX_DEBUG_DO(x) do { x } while (0);
#   define BBOX_DEBUG_SHOW(x) do { std::cerr << #x << ": " << x << std::endl; } while (0)
#   define BBOX_DEBUG_PRINTLN(x) do { std::cerr <<  x << std::endl; } while (0)
#else
#   define BBOX_DEBUG_DO(x)
#   define BBOX_DEBUG_SHOW(x)
#   define BBOX_DEBUG_PRINTLN(x)
#endif



/*!
A simple (axis aligned) bounding box, represented by a min and max point corresponding to the minimal and maximal coordinates of everything contained within the bounding box.
*/
struct BoundingBox
{
    Point min, max; //!< the minimal and maximal coordinates of everything contained within the bounding box.
    //spaceType xmin,xmax,ymin,ymax,zmin,zmax;
//    BoundingBox(spaceType xmin_,spaceType xmax_,spaceType ymin_,spaceType ymax_,spaceType zmin_,spaceType zmax_)
//        : min(xmin_, xmax_, ymin_) , max(ymax_, zmin_, zmax_) {};
    BoundingBox(Point min_, Point max_) : min(min_), max(max_) {};

    BoundingBox() {};

    /*!
    Returns a bounding box with coordinates able to contain both this bounding box and bounding box b.
    */
    BoundingBox(BoundingBox& a, BoundingBox& b)
    : BoundingBox(  Point(std::min(a.min.x, b.min.x),std::min(a.min.y, b.min.y),std::min(a.min.z, b.min.z)),
                    Point(std::max(a.max.x, b.max.x),std::max(a.max.y, b.max.y),std::max(a.max.z, b.max.z)))
    { };

    /*!
    Returns true if the two binding boxes have a voljume as intersection.

    If the intersection consists of a plane, line or vertex it returns false.
    */
    bool intersectsWith(BoundingBox& b) // TODO : what if a.xmax == b.xmin???
    {
        return !(
            (min.x >= b.max.x || max.x <= b.min.x)
            &&
            (min.y >= b.max.y || max.y <= b.min.y)
            &&
            (min.z >= b.max.z || max.z <= b.min.z)
            );
    };

    Point mid() { return (max+min)/2; }; //!< the geometric middle of the box

    template<class CharT, class TraitsT>
    friend
    std::basic_ostream<CharT, TraitsT>&
    operator <<(std::basic_ostream<CharT, TraitsT>& os, const BoundingBox& b)
    {
        return os << b.min << "-" << b.max;
    }
};


#endif // BOUNDING_BOX_H
