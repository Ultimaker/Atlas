#include "HEPMeshVertex.h"

#include "HEPMesh.h"
#include "HEPMeshEdge.h"

#include <set>
#include <algorithm> // for_each


#include "../MACROS.h" // debug
// enable/disable debug output
#define HEP_MESH_DEBUG 0

#define HEP_MESH_DEBUG_SHOW(x) do { std::cerr << #x << ": " << x << std::endl; } while (0)
#define HEP_MESH_DEBUG_PRINTLN(x) DEBUG_PRINTLN(x)
#if HEP_MESH_DEBUG == 1
#  define HEP_MESH_DEBUG_DO(x) DEBUG_DO(x)
#else
#  define HEP_MESH_DEBUG_DO(x)
#endif


HEP_EdgeHandle HEP_VertexHandle::someEdge()
{
    return HEP_EdgeHandle(*m, v.someEdge);
}

Point HEP_VertexHandle::p() { return vertex().p; };


bool HEP_VertexHandle::isManifold(FVMeshVertexHandle& correspondingFVMeshVertex)
{
    if (correspondingFVMeshVertex.vertex().connected_faces.size() < 6) return true;
    // for a correct model, the minimal amount of faces sharing a vertex in a 2D manifold is 3
    // and we need at least 6 faces to connect 2 such manifolds, leading to a non-manifold vertex

    std::vector<std::vector<HEP_EdgeHandle>> groups;
    getConnectedEdgeGroups(correspondingFVMeshVertex, groups);
    return groups.size() == 1;
}

void HEP_VertexHandle::getConnectedEdgeGroups(FVMeshVertexHandle& correspondingFVMeshVertex, std::vector<std::vector<HEP_EdgeHandle>> & result)
{
    HEP_MESH_DEBUG_PRINTLN(" getConnectedEdgeGroups ");
    std::vector<uint32_t>& connected_faces_idx = correspondingFVMeshVertex.vertex().connected_faces;
    std::set<HEP_Face*> checkedFaces;

    for (int f = 0; f < connected_faces_idx.size(); f++)
    {
        if (checkedFaces.find(&m->faces[f]) != checkedFaces.end()) continue; // continue when found

        std::vector<HEP_EdgeHandle> currentGroup;

        HEP_FaceHandle fh(*m, &m->faces[connected_faces_idx[f]]);
        HEP_EdgeHandle outEdgeFirst = fh.getEdgeFrom(*this);

        if((&outEdgeFirst.edge()) == nullptr)
        {
            HEP_MESH_DEBUG_PRINTLN("HEP_VertexHandle::getConnectedEdgeGroups >>> FVMesh doesnt correpsond!" );
            return;
        }

        HEP_EdgeHandle* outEdge = &outEdgeFirst;
        do
        {
            currentGroup.push_back(*outEdge);
            checkedFaces.insert(&outEdge->face().face());
//            HEP_EdgeHandle nxt = outEdge.converse().next();
//            outEdge = nxt;
            outEdge = new HEP_EdgeHandle(outEdge->converse().next());
        } while (*outEdge != outEdgeFirst);

        result.push_back(currentGroup);
    }

}

void HEP_VertexHandle::testGetConnectedEdgeGroups()
{
    FVMesh fvMesh(nullptr);
    Point3 a(-1000,0,0);
    Point3 b(0,-1000,0);
    Point3 c(0,0,0);
    Point3 d(0,0,1000);
    Point3 d2(100,100,1000);
    Point3 e(0,1000,0);
    Point3 f(1000,0,0);
    fvMesh.addFace(b, a, d); // top
    fvMesh.addFace(a, c, b); // bottom
    fvMesh.addFace(a, d, c); // front
    fvMesh.addFace(b, c, d); // back

    fvMesh.addFace(f, d2, e); // top
    fvMesh.addFace(e, c, f); // bottom
    fvMesh.addFace(d2, c, e); // front
    fvMesh.addFace(c, d2, f); // back

    fvMesh.finish();

    HE_Mesh ffMesh(fvMesh);
    HEP_Mesh heMesh(ffMesh);

    heMesh.debugOutputWholeMesh();

    HEP_VertexHandle vh(heMesh, &heMesh.vertices[3]);
    FVMeshVertexHandle vh0(fvMesh, 3);


    HEP_MESH_DEBUG_PRINTLN("starting isManifold computation");
    HEP_MESH_DEBUG_SHOW(vh.isManifold(vh0));


    HEP_MESH_DEBUG_PRINTLN("starting getConnectedEdgeGroups computation");

    std::vector<std::vector<HEP_EdgeHandle>> groups;
    vh.getConnectedEdgeGroups(vh0, groups);

    std::cerr << " groups" << std::endl;
    for_each(groups.begin(), groups.end(),
        [](std::vector<HEP_EdgeHandle> group)
        {
            std::cerr << " next group: ";
            for_each(group.begin(), group.end(), [](HEP_EdgeHandle eh) { std::cerr << long(&eh.e) << " ";} );
            std::cerr << std::endl;
        }
    );

    vh.splitWhenNonManifold(vh0);
    heMesh.debugOutputWholeMesh();
    HEP_MESH_DEBUG_SHOW(&vh);
    HEP_MESH_DEBUG_SHOW(&vh0);
    HEP_MESH_DEBUG_SHOW(vh.isManifold(vh0));

}


void HEP_VertexHandle::splitWhenNonManifold(FVMeshVertexHandle& correspondingFVMeshVertex)
{
    std::vector<std::vector<HEP_EdgeHandle>> groups;
    getConnectedEdgeGroups(correspondingFVMeshVertex, groups);

    if (groups.size() == 1) return;

    for (int g = 1; g < groups.size(); g++)
    {
        std::vector<HEP_EdgeHandle>& group = groups[g];

        FPoint3 direction = FPoint3(group[0].from_vert().p() - group[0].to_vert().p()).normalized();
        Point3 newLoc = group[0].from_vert().p() + (direction * INT2MM(MELD_DISTANCE + 2) ).toPoint3();


        m->vertices.emplace_back(newLoc, &group[0].edge());

        for_each(group.begin(), group.end(), [this](HEP_EdgeHandle edge)
        {
            edge.edge().from_vert = & m->vertices[m->vertices.size()-1];
        });

    }

}













