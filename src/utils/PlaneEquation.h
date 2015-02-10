/** Copyright (C) 2013 David Braam - Released under terms of the AGPLv3 License */
#ifndef PLANE_EQUATION_H
#define PLANE_EQUATION_H


#include <iostream>
#include <stdint.h>
#include <math.h>

#include "floatpoint.h"



template<typename Pt2D>
struct DefaultCoordGetter2D
{
    static double x(Pt2D a) { return a.x(); };
    static double y(Pt2D a) { return a.y(); };
};

//typedef FPoint Vec3;
struct Vec2 : public std::pair<double, double>
{
    double x() const { return first; } ;
    double y() const { return second; } ;

    Vec2(double a, double b) : std::pair<double,double>(a,b) {};

    template<class CharT, class TraitsT>
    friend
    std::basic_ostream<CharT, TraitsT>&
    operator <<(std::basic_ostream<CharT, TraitsT>& os, const Vec2& p)
    {
        return os << "(" << p.x() << ", " << p.y() << ")";
    }
};

template<typename Vec3, typename Vec2_ = Vec2, typename CoordGetter = DefaultCoordGetter2D<Vec2_>>
class PlaneEquation
{
public:
    Vec3 x_axis, y_axis;
    Vec3 origin;

    static inline Vec3 project_on_line(Vec3 a, Vec3 on) {
        if (a.vSize2()==0)
            std::cerr << "WARNING! orthogonalizing vector with zero length!" << std::endl;
        Vec3 ret = a.dot(on) / on.vSize2() * on;
        return ret;
    }

    static inline Vec3 orthogonalize(Vec3 b, Vec3 a)
    {
        return b - project_on_line(b,a);
    };

    PlaneEquation(Vec3 axis_1, Vec3 b) : x_axis(axis_1.normalized()), y_axis(orthogonalize(b,axis_1).normalized()), origin(0,0,0) { };
    PlaneEquation(Vec3 a, Vec3 b, Vec3 c) : x_axis((b-a).normalized()), y_axis(orthogonalize( c - a, b - a).normalized()), origin(a) { };

    Vec2_ project(Vec3 p)
    {
        double dist_to_plane = p.dot(x_axis.cross(y_axis));
        if (dist_to_plane > .1)
            std::cerr << "WARNING! Projecting point to plane with a distance of " << dist_to_plane << "!" << std::endl;
        return Vec2_( (p-origin).dot(x_axis), (p-origin).dot(y_axis) );
    };
    Vec3 project(Vec2_ p)
    {
        return x_axis * CoordGetter::x(p) + y_axis * CoordGetter::y(p);
    }

    static void test()
    {
        PlaneEquation<FPoint3> pe(FPoint3(0,0,1),FPoint3(1,1,1));
        std::cerr << pe.x_axis << " _|_ " << pe.y_axis << std::endl;
        std::cerr << pe.project(FPoint3(10,10,10)) << std::endl;
        std::cerr << pe.project(pe.project(FPoint3(10,11,10))) << std::endl;
    };
};

#endif//INT_POINT_H
