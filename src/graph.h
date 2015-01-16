#ifndef GRAPH_H_INCLUDED
#define GRAPH_H_INCLUDED



#endif // GRAPH_H_INCLUDED



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



class Graph
{
std::vector<Node> nodes;
std::vector<Arrow> arrows;

};
