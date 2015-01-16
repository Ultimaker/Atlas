#ifndef GRAPH_H_INCLUDED
#define GRAPH_H_INCLUDED

#include <vector>


template<class NodeT, class ArrowT>
class Graph
{
public:
    struct Arrow;
    struct Node
    {
        NodeT data;
        Arrow *first_in, *last_in, *first_out, *last_out;
    };

    struct Arrow
    {
        ArrowT data;
        Node *from, *to;
        Arrow *prev_same_from, *next_same_from,
              *prev_same_to, *next_same_to;
    };

    std::vector<Node> nodes;
    std::vector<Arrow> arrows;

    void connect(Node& a, Node& b)
    {
        arrows.emplace_back();
        Arrow* newArrow = &arrows[arrows.size()-1];
        newArrow.from = &a;
        newArrow.to = &b;

        a.last_out->next_same_from = newArrow;
        newArrow.prev_same_from = a.last_out;
        a.last_out = newArrow;

        b.last_in->next_same_to = newArrow;
        newArrow.prev_same_to = b.last_in;
        b.last_in = newArrow;
    };

};



#endif // GRAPH_H_INCLUDED
