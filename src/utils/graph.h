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
        bool points() { return first_out != nullptr; }
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

        newArrow.prev_same_from = a.last_out;
        if (a.last_out == nullptr)
        {
            a.first_out = newArrow;
            a.last_out = newArrow;
        }
        else
        {
            a.last_out->next_same_from = newArrow;
            a.last_out = newArrow;
        }

        newArrow.prev_same_to = b.last_in;
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
    };

    void disconnect(std::vector<Arrow>::iterator  a)
    {
        a->prev_same_from.next_same_from = a->next_same_from;
        a->next_same_from.prev_same_from = a->prev_same_from;

        a->prev_same_to.next_same_to = a->next_same_to;
        a->next_same_to.prev_same_to = a->prev_same_to;

        arrows.erase(a);
    };

};



#endif // GRAPH_H_INCLUDED
