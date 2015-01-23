#ifndef GRAPH_H_INCLUDED
#define GRAPH_H_INCLUDED

#include <vector>

#include <iostream> // std::cerr

#include "../Kernel.h"

// enable/disable debug output
#define GRAPH_DEBUG 1

#if GRAPH_DEBUG == 1
#   define GRAPH_DEBUG_DO(x) do { x } while (0);
#   define GRAPH_DEBUG_SHOW(x) do { std::cerr << #x << " = " << x << std::endl; } while (0)
#   define GRAPH_DEBUG_PRINTLN(x) do { std::cerr <<  x << std::endl; } while (0)
#else
#   define GRAPH_DEBUG_DO(x)
#   define GRAPH_DEBUG_SHOW(x)
#   define GRAPH_DEBUG_PRINTLN(x)
#endif


/*!
Directed graph datatype.
Nodes and arrows can be annotated with NodeT and ArrowT objects
*/

template<class NodeT, class ArrowT>
class Graph
{
public:
    struct Arrow;
    struct Node
    {
        NodeT data;
        Arrow *first_in, *last_in, *first_out, *last_out;
        bool points() { return first_out != nullptr; }
        Node(NodeT& data_) : data(data_), first_in(nullptr), last_in(nullptr), first_out(nullptr), last_out(nullptr) {};
        Node& operator=(const Node& b)
        {
            GRAPH_DEBUG_PRINTLN("changing Node data!");
            data = b.data; first_in = b.first_in; last_in = b.last_in; first_out=b.first_out; last_out=b.last_out;
            return *this;
        }
    };

    struct Arrow
    {
        ArrowT data;
        Node *from, *to;
        Arrow *prev_same_from, *next_same_from,
              *prev_same_to, *next_same_to;
        Arrow(Node* from_, ArrowT data_, Node* to_) : data(data_), from(from_), to(to_), prev_same_from(nullptr), next_same_from(nullptr), prev_same_to(nullptr), next_same_to(nullptr) {};
    };

//protected:
    std::vector<Node*> nodes; // all nodes in arbitrary order
    std::vector<Arrow*> arrows; // all arrows in arbitrary order

public:
    Arrow* connect(Node& a, Node& b, ArrowT data)
    {
        DEBUG_HERE;
        Arrow* newArrow = new Arrow(&a, data, &b);
        arrows.push_back(newArrow);
        DEBUG_HERE;
        GRAPH_DEBUG_PRINTLN(long(&a));
        GRAPH_DEBUG_PRINTLN(a.last_out);

        newArrow->prev_same_from = a.last_out;
        DEBUG_HERE;
        if (a.last_out == nullptr)
        {
            DEBUG_HERE;
            a.first_out = newArrow;
            DEBUG_HERE;
            a.last_out = newArrow;
        }
        else
        {
            DEBUG_HERE;
            a.last_out->next_same_from = newArrow;
            DEBUG_HERE;
            a.last_out = newArrow;
        }
        DEBUG_HERE;

        newArrow->prev_same_to = b.last_in;
        if (b.last_in == nullptr)
        {
            b.first_in = newArrow;
            b.last_in = newArrow;
        }
        else
        {
            b.last_in->next_same_to = newArrow;
            b.last_in = newArrow;
        }
        DEBUG_HERE;
        return newArrow;
    };

    Node* addNode(NodeT& data)
    {
        Node* newNode = new Node(data);
        nodes.push_back(newNode);
        return newNode;
    };



    void disconnect(typename std::vector<Arrow>::iterator  a)
    {
        a->prev_same_from.next_same_from = a->next_same_from;
        a->next_same_from.prev_same_from = a->prev_same_from;

        a->prev_same_to.next_same_to = a->next_same_to;
        a->next_same_to.prev_same_to = a->prev_same_to;

        arrows.erase(a);
        delete &*a;
    };

    ~Graph()
    {
        nodes.clear();
        arrows.clear();
    }
};



#endif // GRAPH_H_INCLUDED
