#ifndef EAE_GRAPH_H
#define EAE_GRAPH_H

#include "eae.hh"

using namespace Stg;
using namespace std;

/**
 * Namespace for energy aware exploration (eae).
 */
namespace eae
{
    /**
     * A class for the edges of the graph.
     */
    class Edge
    {
    public:
        /**
         * Constructor.
         *
         * @param Node* to:
         * @param double cost:
         */
        Edge(Node* to, double cost=1.0);

        /**
         *
         */
        Node* to;

        /**
         *
         */
        double cost;
    };

    /**
     * A class for the nodes of the graph.
     */
    class Node
    {
    public:
        /**
         * Constructor.
         */
        Node(Pose pose, double value=0);

        /**
         * Destructor.
         */
        ~Node();

        /**
         *
         */
        void AddEdge(Edge* edge);

        /**
         *
         */
        void Draw() const;

        /**
         *
         */
        Pose pose;

        /**
         *
         */
        double value;

        /**
         *
         */
        std::vector<Edge*> edges;
    };

    /**
     * A class for the graph representing the map frontiers.
     */
    class Graph
    {
    public:
        /**
         * Constructor.
         */
        Graph();

        /**
         * Destructor.
         */
        ~Graph();

        /**
         *
         */
        void AddNode(Node* node);

        /**
         *
         */
        Node* PopFront();

        /**
         *
         */
        Node* PopBack();

        /**
         *
         */
        void Draw() const;

        /**
         * Find the node with the lowest value within range of the given
         * pose and return the absolute heading from pose to that node.
         */
        bool GoodDirection(const Pose& pose, meters_t range, radians_t& heading_result);

        /**
         * Get the size of the graph.
         *
         * @retrun unsigned int: The size of the graph in number of nodes.
         */
        unsigned int Size();

        /**
         *
         */
        std::vector<Node*> nodes;
    };

    /**
     * A class for visualizing the graph.
     */
    class GraphVis : public Visualizer
    {
    public:
        /**
         * Constructor.
         */
        GraphVis(Graph** graphpp);

        /**
         * Destructor.
         */
        virtual ~GraphVis();

        /**
         *
         */
        virtual void Visualize(Model* mod, Camera* cam);

        /**
         *
         */
        Graph** graphpp;
    };
}

#endif
