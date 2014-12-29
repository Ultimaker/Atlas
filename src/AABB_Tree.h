#ifndef AABB_TREE_H
#define AABB_TREE_H

#include <algorithm> // min, max
#include "utils/intpoint.h" // Point3

#include "Kernel.h"

#include "BoundingBox.h"

// enable/disable debug output
#define AABB_DEBUG 0

#if AABB_DEBUG == 1
#   define AABB_DEBUG_DO(x) do { x } while (0);
#   define AABB_DEBUG_SHOW(x) do { std::cerr << #x << " = " << x << std::endl; } while (0)
#   define AABB_DEBUG_PRINTLN(x) do { std::cerr <<  x << std::endl; } while (0)
#else
#   define AABB_DEBUG_DO(x)
#   define AABB_DEBUG_SHOW(x)
#   define AABB_DEBUG_PRINTLN(x)
#endif



/*!
Axis Aligned Bounding Box search tree.
@param T the type of object to be stored with/in the bounding boxes of leaves.
*/
template<typename T>
class AABB_Tree
{
public:

    static void test()
    {
        typedef AABB_Tree<int>::Node Node;
        std::vector<Node*> leaves;

        int ints[9] = {0,1,2,3,4,5,6,7,8};

        for (int i = 0 ; i < 9 ; i++)
        {
            Point p_a(i,i,i);
            Point3 p_b(i+1,i+1,i+1);
            leaves.push_back(new Node(BoundingBox(p_a, p_b),&ints[i]));
        }
        std::cerr << " TEST building AABB_Tree... " << std::endl;

        std::reverse(leaves.begin(), leaves.end());

        AABB_Tree<int> tree(leaves);

        tree.debugPrint();

        std::cerr << " TEST intersecting AABB_Tree... " << std::endl;

        std::vector<int*> intersections;
        Point p_a(2,3,4);
        Point3 p_b(5,6,7);
        BoundingBox bbox(p_a, p_b);
        tree.getIntersections(bbox, intersections);

        for (int* i : intersections)
            std::cerr << *i << " ";
        std::cerr << std::endl;

    }

    struct Node
    {
        BoundingBox box;
        T* object; // null pointer if Fork-Node

        Node(BoundingBox b, T* o) : box(b), object(o) {};
        int pos; //!< position in the list allnodes
    protected:
    };

    std::vector<Node*> allnodes; //!< allNodes[i].childA = allnodes[2*i+1] ; allNodes[i].childB = allnodes[2*i+2] ; allNodes[i].parent = allNodes[(i-1)/2]
    int depth; //!< the depth of the tree


    AABB_Tree(std::vector<Node*>& objects)
    {
        if (objects.size() == 0) return;

        int leavesSize = 1;
        int totalSize = 1;
        depth = 0;
        while (leavesSize < objects.size())
        {
            leavesSize *= 2;
            totalSize += leavesSize;
            depth++;
        }

        allnodes.resize(totalSize);

//        std::cerr << "treeSize = " << leavesSize << std::endl;
//        std::cerr << "tree_list_pos , dim , left , right , depth " << std::endl;

        construct(objects, 0, 0, 0, objects.size()-1);


    };
    ~AABB_Tree()
    {
        allnodes.clear();
    };

    void debugPrint()
    {

        AABB_DEBUG_PRINTLN("  allnodes.size()  = "<<  allnodes.size() );

        int width = 1;
        int last_depth = 0;
        int d = 0;
        for (int i = 0 ; i < allnodes.size() ; i++)
        {
            if (allnodes[i] == nullptr)
            {
                std::cerr << "  _  " ;
                continue;
            }
                std:: cerr << allnodes[i]->box.min <<"-"<<allnodes[i]->box.max <<"   ";
            if (allnodes[i]->object != nullptr) std:: cerr << * allnodes[i]->object << "   ";

            if (i == last_depth)
            {
                std:: cerr << std::endl;
                width *= 2;
                last_depth += width;
                d++;
            }
        }

        std:: cerr << std::endl;
    }

protected:
    Node* construct(std::vector<Node*>& objects, int tree_list_pos, int dim, int left, int right)
    {
            AABB_DEBUG_PRINTLN(tree_list_pos << ", " << dim << ", " << left << ", " << right);
            AABB_DEBUG_DO(
                for (int i = 0 ; i < objects.size() ; i++)
                {
                    if (i == left) std::cerr << ">";
                    std::cerr << * objects[i]->object;
                    if (i == right) std::cerr << "<";

                }
                std::cerr << std::endl;
            )

        if (right == left)
        {
            allnodes[tree_list_pos] = objects[left];
            allnodes[tree_list_pos]->pos = tree_list_pos;
            AABB_DEBUG_PRINTLN("inserted " << * allnodes[tree_list_pos]->object << " at position " << tree_list_pos);
            return allnodes[tree_list_pos];
        } else
        {
            int mid = select(objects, left, right, (left + right)/2,
                    [dim](Node& n)
                    {
                        Point mid = n.box.mid();
                        spaceType ret = (dim==0)? mid.x : (dim==1)? mid.y : mid.z;
                        return ret;
                    }
                );
            AABB_DEBUG_SHOW(mid);

            Node* left_node = construct(objects, tree_list_pos*2+1, (dim+1) % 3, left, mid);
            AABB_DEBUG_PRINTLN(" right... ");
            Node* right_node = construct(objects, tree_list_pos*2+2, (dim+1) % 3, mid+1, right);

            allnodes[tree_list_pos] = new Node(BoundingBox(left_node->box, right_node->box), nullptr);
            allnodes[tree_list_pos]->pos = tree_list_pos;
            return allnodes[tree_list_pos];
        }
    }

public:
    bool isLeaf(Node& p) // non-leaves must have both children!
    {
        int pos = p.pos*2 +2; // check only if it has a right child
        if (pos >= allnodes.size()) return true;
        else return allnodes[pos] == nullptr;
    };

    Node* getRoot()
    {
        return (allnodes.size() > 0)? allnodes[0] : nullptr;
    };
    Node* getLeftChild(Node& p)
    {
        int pos = p.pos*2 +1;
        if (pos >= allnodes.size()) return nullptr;
        else return allnodes[pos];
    };
    Node* getRightChild(Node& p)
    {
        int pos = p.pos*2 +2;
        if (pos >= allnodes.size()) return nullptr;
        else return allnodes[pos];
    };
    Node* getParent(Node& c)
    {
        if (c.pos == 0) return nullptr;
        else return allnodes[(c.pos - 1)/2];
    }

    void getIntersections(BoundingBox& bbox, std::vector<T*>& result)
    {
        getIntersections(bbox, result, getRoot());
    }
protected:
    void getIntersections(BoundingBox& bbox, std::vector<T*>& result, Node* node)
    {
        if (node->object == nullptr && node->box.intersectsWith(bbox))
        {
            getIntersections(bbox, result, getLeftChild(*node));
            getIntersections(bbox, result, getRightChild(*node));
        } else
        if (node->box.intersectsWith(bbox))
        {
            result.push_back(node->object);
        }
    }
private:
    template<typename GetVal>
    static int partition(std::vector<Node*>& list, int left, int right, int pivotIndex, GetVal getVal)
    {
        AABB_DEBUG_DO(std::cerr << " :: "; for (int i=0;i<list.size();i++) std::cerr << *list[i]->object; std::cerr << std::endl;)
        spaceType pivotValue = getVal(*list[pivotIndex]);
        std::swap(list[pivotIndex], list[right]);  // Move pivot to end
        int storeIndex = left;
        for (int i = left; i < right ; i++)
        {
            if (getVal(*list[i]) < pivotValue)
            {
                std::swap(list[storeIndex], list[i]);
                storeIndex++;
            }
            AABB_DEBUG_DO(std::cerr << " :: "; for (int i=0;i<list.size();i++) std::cerr << *list[i]->object; std::cerr << " : " << pivotValue << " , " << pivotIndex << std::endl;)
        }
        std::swap(list[right], list[storeIndex]);  // Move pivot to its final place

        AABB_DEBUG_DO(std::cerr << " :: "; for (int i=0;i<list.size();i++) std::cerr << *list[i]->object; std::cerr << std::endl;)

        return storeIndex;
    };
    template<typename GetVal>
    static int select(std::vector<Node*>& list, int left, int right, int n, GetVal getVal)
    {
         if (left == right)
             return left;
         while (true)
         {
             int pivotIndex = (right+left)/2;     // select pivotIndex between left and right
             pivotIndex = partition(list, left, right, pivotIndex, getVal);
             AABB_DEBUG_PRINTLN(" left = " << left << " right = " << right << " pivotIndex = " << pivotIndex <<"\t, \tn = " << n);

             if (n == pivotIndex)       return n;
             else if (n < pivotIndex)   right = pivotIndex - 1;
             else                       left = pivotIndex + 1;
        }
    }
//    spaceType getDistance(BoundingBox a, BoundingBox b)
//    {
//
//    };
};

#endif // AABB_TREE_H
