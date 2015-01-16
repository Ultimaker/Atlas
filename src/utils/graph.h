#ifndef GRAPH_H_INCLUDED
#define GRAPH_H_INCLUDED



#endif // GRAPH_H_INCLUDED

template<class NodeT, class ArrowT>
struct Arrow;

template<class NodeT, class ArrowT>
struct Node
{
    NodeT data;
    Arrow<ArrowT> *first_in, *last_in, *first_out, *last_out;
};

template<class NodeT, class ArrowT>
struct Arrow
{
    ArrowT data;
    Node<NodeT> *from, *to;
    Arrow<ArrowT> *prev_same_from, *next_same_from,
          *prev_same_to, *next_same_to;
};


template<class NodeT, class ArrowT>
class Graph
{
public:

    std::vector<Node<NoteT>> nodes;
    std::vector<Arrow<ArrowT>> arrows;



};
