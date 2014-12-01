/** Copyright (C) 2013 David Braam - Released under terms of the AGPLv3 License */
#ifndef MODELFILE_H
#define MODELFILE_H
/**
modelFile contains the model loaders for the slicer. The model loader turns any format that it can read into a list of triangles with 3 X/Y/Z points.

The format returned is a Model class with an array of faces, which have integer points with a resolution of 1 micron. Giving a maximum object size of 4 meters.
**/

//#include "../mesh.h"
#include "../polyhedra.h"
//#include "../settings.h"

#define MM2INT(n) (int64_t((n) * 1000))

using namespace CGAL;

typedef Vector_3<Kernel> Vector3;
typedef Polyhedron::Vertex_iterator Vertex_iterator;

//A PrintObject is a 3D model with 1 or more 3D meshes.
class PrintObject //: public SettingsBase
{
public:
    std::vector<Polyhedron> meshes;

//    PrintObject(SettingsBase* settings_base)
//    : SettingsBase(settings_base)
//    {
//    }

    Point min() //! minimal corner of bounding box
    {
        if (meshes.size() < 1)
            return Point(0, 0, 0);
        Point first = PolyhedraUtils::min(meshes[0]);
        prim_data x = first.x();
        prim_data y = first.y();
        prim_data z = first.z();
        for(unsigned int i=1; i<meshes.size(); i++)
        {
            Point v = PolyhedraUtils::min(meshes[i]);
            x = std::min(x, v.x());
            y = std::min(y, v.y());
            z = std::min(z, v.z());
        }
        return Point(x,y,z);
    }

    Point max() //! minimal corner of bounding box
    {
        if (meshes.size() < 1)
            return Point(0, 0, 0);
        Point first = PolyhedraUtils::max(meshes[0]);
        prim_data x = first.x();
        prim_data y = first.y();
        prim_data z = first.z();
        for(unsigned int i=1; i<meshes.size(); i++)
        {
            Point v = PolyhedraUtils::max(meshes[i]);
            x = std::max(x, v.x());
            y = std::max(y, v.y());
            z = std::max(z, v.z());
        }
        return Point(x,y,z);
    }

    void clear()
    {
        for(Polyhedron& m : meshes)
            m.clear();
    }

    void offset(Vector3& offset)
    {
        for(Polyhedron& m : meshes)
            for(Vertex_iterator i = m.vertices_begin() ; i != m.vertices_end() ; i++)
                i->point().transform(Aff_transformation_3<Kernel>(Translation(), offset));
    }

    void finalize()
    {
        Point object_min = min();
        Point object_max = max();
        Vector3 object_size = object_max - object_min;
        Vector3 object_offset = Vector3(-object_min.x() - object_size.x() / 2, -object_min.y() - object_size.y() / 2, -object_min.z());
//        object_offset.x += getSettingInt("position.X");
//        object_offset.y += getSettingInt("position.Y");
//        object_offset.z += getSettingInt("position.Z");
        offset(object_offset);
    }
};

class FMatrix3x3_new
{
public:
    double m[3][3];

    FMatrix3x3_new()
    {
        m[0][0] = 1.0;
        m[1][0] = 0.0;
        m[2][0] = 0.0;
        m[0][1] = 0.0;
        m[1][1] = 1.0;
        m[2][1] = 0.0;
        m[0][2] = 0.0;
        m[1][2] = 0.0;
        m[2][2] = 1.0;
    }

    template <class K>
    Point apply(Point_3<K> p)
    {
        return Point(
            MM2INT(p.x() * m[0][0] + p.y() * m[1][0] + p.z() * m[2][0]),
            MM2INT(p.x() * m[0][1] + p.y() * m[1][1] + p.z() * m[2][1]),
            MM2INT(p.x() * m[0][2] + p.y() * m[1][2] + p.z() * m[2][2]));
    }
};

bool loadPolyhedronFromFile(PrintObject* object, const char* filename, FMatrix3x3_new& matrix);

#endif//MODELFILE_H
