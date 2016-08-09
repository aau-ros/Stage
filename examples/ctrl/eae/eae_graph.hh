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
         * @param Node* to: The node the edge is pointing to.
         * @param double cost: The cost of traversing the edge, default 1.
         */
        Edge(Node* to, double cost=1.0);

        /**
         * The node the edge is pointing to.
         */
        Node* to;

        /**
         * The cost of traversing the edge.
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
         *
         * @param Pose pose: The position of the node.
         * @param double value: The value of the node, default 0.
         */
        Node(Pose pose, double value=0);

        /**
         * Destructor.
         */
        ~Node();

        /**
         * Add an edge.
         *
         * @param Edge* edge: The edge.
         */
        void AddEdge(Edge* edge);

        /**
         * Draw the node.
         */
        void Draw() const;

        /**
         * Position of the node.
         */
        Pose pose;

        /**
         * Value of the node.
         */
        double value;

        /**
         * All edges connected to the node.
         */
        vector<Edge*> edges;
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
         * Add a node to the graph.
         *
         * @param Node* node: The node.
         */
        void AddNode(Node* node);

        /**
         * Remove a node from the beginning and return it.
         *
         * @return Node*: The node that has been removed.
         */
        Node* PopFront();

        /**
         * Remove a node from the end and return it.
         *
         * @return Node*: The node that has been removed.
         */
        Node* PopBack();

        /**
         * Draw the graph.
         */
        void Draw() const;

        /**
         * Find the node with the lowest value within range of the given
         * pose and return the absolute heading from pose to that node.
         *
         * @param const Pose& pose: Current position.
         * @param meters_t range: Maximum distance to node.
         * @param radians_t& heading_result: Returns the heading to the best node.
         *
         * @return bool: True, if a node is found, false otherwise.
         */
        bool GoodDirection(const Pose& pose, meters_t range, radians_t& heading_result);

        /**
         * Get the size of the graph.
         *
         * @return unsigned int: The size of the graph in number of nodes.
         */
        unsigned int Size();

        /**
         * All the nodes in the graph.
         */
        vector<Node*> nodes;
    };

    /**
     * A class for visualizing the graph.
     */
    class GraphVis : public Visualizer
    {
    public:
        /**
         * Constructor.
         *
         * @param Graph** graphpp: The graph to visualize.
         */
        GraphVis(Graph** graphpp);

        /**
         * Destructor.
         */
        virtual ~GraphVis();

        /**
         * Visualize the graph.
         *
         * @param Model* mod: The model for visualization.
         * @param Camera* cam: The camera for visualization.
         */
        virtual void Visualize(Model* mod, Camera* cam);

        /**
         * The graph to visualize.
         */
        Graph** graphpp;
    };
}

#endif
