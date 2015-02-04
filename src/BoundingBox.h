#ifndef BOUNDING_BOX_H
#define BOUNDING_BOX_H

#include <algorithm> // min, max
#include "utils/intpoint.h" // Point3

#include "Kernel.h"


#include "MACROS.h" // debug
// enable/disable debug output
#define BBOX_DEBUG 0

#if BBOX_DEBUG == 1
#   define BBOX_DEBUG_DO(x) DEBUG_DO(x)
#   define BBOX_DEBUG_SHOW(x) DEBUG_SHOW(x)
#   define BBOX_DEBUG_PRINTLN(x) DEBUG_PRINTLN(x)
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

    BoundingBox(spaceType minx, spaceType miny, spaceType minz, spaceType maxx, spaceType maxy, spaceType maxz)
    : min(minx, miny, minz)
    , max(maxx, maxy, maxz)
    {};

    /*!
    Returns a bounding box containing both points
    */
    BoundingBox(const Point& a, const Point& b)
    : min(std::min(a.x, b.x),std::min(a.y, b.y),std::min(a.z, b.z))
    , max(std::max(a.x, b.x),std::max(a.y, b.y),std::max(a.z, b.z))
    { };

//        : min(xmin_, xmax_, ymin_) , max(ymax_, zmin_, zmax_) {};

    BoundingBox() {};

    /*!
    Returns a bounding box with coordinates able to contain both this bounding box and bounding box b.
    */
    BoundingBox(const BoundingBox& a, const BoundingBox& b)
    : min(std::min(a.min.x, b.min.x),std::min(a.min.y, b.min.y),std::min(a.min.z, b.min.z))
    , max(std::max(a.max.x, b.max.x),std::max(a.max.y, b.max.y),std::max(a.max.z, b.max.z))
    { };

    BoundingBox operator +(const BoundingBox& b)
    {
        return BoundingBox(*this, b);
    }

    BoundingBox operator +(const Point& b)
    {
    return BoundingBox( std::min(min.x, b.x),std::min(min.y, b.y),std::min(min.z, b.z),
                        std::max(max.x, b.x),std::max(max.y, b.y),std::max(max.z, b.z)  );
    }
    void operator +=(const Point& b)
    {
        min.x = std::min(min.x, b.x);
        min.y = std::min(min.y, b.y);
        min.z = std::min(min.z, b.z);
        max.x = std::max(max.x, b.x);
        max.y = std::max(max.y, b.y);
        max.z = std::max(max.z, b.z);
    }

    void offset(const Point& v)
    {
        min += v;
        max += v;
    }

    /*!
    Returns true if the two binding boxes have a voljume as intersection.

    If the intersection consists of a plane, line or vertex it returns false.
    */
    bool intersectsWith(const BoundingBox& b) // TODO : what if a.xmax == b.xmin???
    {
        return !(
            (min.x >= b.max.x || max.x <= b.min.x)
            ||
            (min.y >= b.max.y || max.y <= b.min.y)
            ||
            (min.z >= b.max.z || max.z <= b.min.z)
            );
    };

    Point mid() { return (max+min)/2; }; //!< the geometric middle of the box

    Point size() { return max-min; }; //!< a point containing the width, height and depth information

    template<class CharT, class TraitsT>
    friend
    std::basic_ostream<CharT, TraitsT>&
    operator <<(std::basic_ostream<CharT, TraitsT>& os, const BoundingBox& b)
    {
        return os << b.min << "-" << b.max;
    }
};


#endif // BOUNDING_BOX_H
