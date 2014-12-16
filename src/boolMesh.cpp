#include "boolMesh.h"


void BooleanFVMeshOps::subtract(FVMesh& keep, HE_Mesh& subtracted, FVMesh& result)
{
//! is more efficient when keep is smaller than subtracted.

/*
1) Find every pair of triangles that intersect
2) Compute the intersections as segments
3) For each intersected triangle, store the intersection segments and orient them
4) Triangulate the intersected facets and add to the solution the triangles that belong to it.
5) Propagate the result to the whole polyhedra, using the connectivity of the facets
*/

    // : \/ construct an AABB tree from the keep-mesh
    typedef AABB_Tree<FVMeshFace>::Node Node;

    std::vector<Node*> leaves;
    for (int f = 0; f < keep.faces.size(); f++)
        leaves.push_back(new Node(keep.computeFaceBbox(f), &keep.faces[f]));

    AABB_Tree<FVMeshFace> keep_aabb(leaves);

    for (int f = 0 ; f < subtracted.faces.size() ; f++)
    {
        BoundingBox tribbox = subtracted.computeFaceBbox(f);
        std::vector<FVMeshFace*> intersectingFaces;
        keep_aabb.getIntersections(tribbox, intersectingFaces);

        HE_Face& face = subtracted.faces[f];

    }

}
