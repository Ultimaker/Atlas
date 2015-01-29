#include "HalfEdgeMeshVertex.h"

#include "HalfEdgeMesh.h"
#include "HalfEdgeMeshEdge.h"

#include "FVMeshVertex.h"


#include <algorithm> // for_each

#include <set>


#include "../MACROS.h" // debug
// enable/disable debug output
#define HE_MESH_DEBUG 1

#define HE_MESH_DEBUG_SHOW(x) DEBUG_SHOW(x)
#define HE_MESH_DEBUG_PRINTLN(x) DEBUG_PRINTLN(x)
#if HE_MESH_DEBUG == 1
#  define HE_MESH_DEBUG_DO(x) DEBUG_DO(x)
#else
#  define HE_MESH_DEBUG_DO(x)
#endif


HE_EdgeHandle HE_VertexHandle::someEdge()
{
    return HE_EdgeHandle(*m, m->vertices[idx].someEdge_idx);
}

Point& HE_VertexHandle::p() { return vertex().p; };

HE_Vertex& HE_VertexHandle::vertex()
{
    return m->vertices[idx];
};


bool HE_VertexHandle::isManifold(FVMeshVertexHandle& correspondingFVMeshVertex)
{
    if (correspondingFVMeshVertex.vertex().connected_faces.size() < 6) return true;
    // for a correct model, the minimal amount of faces sharing a vertex in a 2D manifold is 3
    // and we need at least 6 faces to connect 2 such manifolds, leading to a non-manifold vertex

    std::vector<std::vector<HE_EdgeHandle>> groups;
    getConnectedEdgeGroups(correspondingFVMeshVertex, groups);
    return groups.size() == 1;
}

void HE_VertexHandle::getConnectedEdgeGroups(FVMeshVertexHandle& correspondingFVMeshVertex, std::vector<std::vector<HE_EdgeHandle>> & result)
{
    std::vector<uint32_t>& connected_faces_idx = correspondingFVMeshVertex.vertex().connected_faces;
    std::set<uint32_t> checkedFaces;

    for (int f = 0; f < connected_faces_idx.size(); f++)
    {
        if (checkedFaces.find(connected_faces_idx[f]) != checkedFaces.end()) continue;

        std::vector<HE_EdgeHandle> currentGroup;

        HE_FaceHandle fh(*m, connected_faces_idx[f]);
        HE_EdgeHandle outEdgeFirst = fh.getEdgeFrom(*this);

        HE_EdgeHandle outEdge = outEdgeFirst;
        int counter = 0;
        do
        {
            if (counter > MAX_EDGES_PER_VERTEX)
            {
                std::cout << "too many edges on vertex!!" << std::endl;
                        std::cout << outEdgeFirst.idx << std::endl;

                        int c2 = 0;
                        do
                        {
                            if (++c2 > 20) break;
                            std::cout << outEdge.idx << std::endl;
                            outEdge = outEdge.converse().next();
                        } while (outEdge != outEdgeFirst);
                break;
            }
            currentGroup.push_back(outEdge);
            checkedFaces.insert(outEdge.face().idx);
//            HE_EdgeHandle nxt = outEdge.converse().next();
//            outEdge = nxt;
            outEdge = outEdge.converse().next(); counter++;
        } while (outEdge != outEdgeFirst);

        result.push_back(currentGroup);
    }

}

void HE_VertexHandle::testGetConnectedEdgeGroups()
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

    HE_Mesh heMesh(fvMesh);

    heMesh.debugOutputWholeMesh();

    HE_VertexHandle vh(heMesh, 3);
    FVMeshVertexHandle vh0(fvMesh, 3);


    HE_MESH_DEBUG_PRINTLN("starting isManifold computation");
    HE_MESH_DEBUG_SHOW(vh.isManifold(vh0));


    HE_MESH_DEBUG_PRINTLN("starting getConnectedEdgeGroups computation");

    std::vector<std::vector<HE_EdgeHandle>> groups;
    vh.getConnectedEdgeGroups(vh0, groups);

    std::cerr << " groups" << std::endl;
    for_each(groups.begin(), groups.end(),
        [](std::vector<HE_EdgeHandle> group)
        {
            std::cerr << " next group: ";
            for_each(group.begin(), group.end(), [](HE_EdgeHandle eh) { std::cerr << eh.idx << " ";} );
            std::cerr << std::endl;
        }
    );

    vh.splitWhenNonManifold(vh0);

}



void HE_VertexHandle::splitWhenNonManifold(FVMeshVertexHandle& correspondingFVMeshVertex)
{
    std::vector<std::vector<HE_EdgeHandle>> groups;
    getConnectedEdgeGroups(correspondingFVMeshVertex, groups);

    if (groups.size() == 1) return;


    for (int g = 1; g < groups.size(); g++)
    {
        std::vector<HE_EdgeHandle>& group = groups[g];

        FPoint3 direction = FPoint3(group[0].from_vert().p() - group[0].to_vert().p()).normalized();

        Point3 newLoc = group[0].from_vert().p() + (direction * INT2MM(MELD_DISTANCE + 2) ).toPoint3();

        m->vertices.emplace_back(newLoc, group[0].idx);

        for_each(group.begin(), group.end(), [this](HE_EdgeHandle edge)
        {
            edge.edge().from_vert_idx = m->vertices.size()-1;
        });

    }

    DEBUG_HERE;
}

