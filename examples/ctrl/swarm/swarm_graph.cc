#include "swarm_graph.hh"

using namespace Stg;
using namespace std;

namespace swarm
{
    Edge::Edge(Node* to, double cost)
    {
        this->to = to;
        this->cost = cost;
    }

    Node::Node(Pose pose, double value){
        this->pose = pose;
        this->value = value;
    }

    Node::~Node()
    {
        FOR_EACH(it, edges){
            delete *it;
        }
    }

    void Node::AddEdge(Edge* edge)
    {
        assert(edge);
        edges.push_back(edge);
    }

    void Node::Draw() const
    {
        glBegin(GL_LINES);
        FOR_EACH(it, edges){
            glVertex2f(pose.x, pose.y);
            glVertex2f((*it)->to->pose.x, (*it)->to->pose.y);
        }
        glEnd();
    }

    Graph::Graph(int robot){
        this->robot = robot;
    }

    Graph::~Graph()
    {
        FOR_EACH(it, nodes){
            delete *it;
        }
    }

    void Graph::AddNode(Node* node){
        nodes.push_back(node);
    }

    Node* Graph::PopFront()
    {
        const std::vector<Node*>::iterator& it = nodes.begin();
        nodes.erase(it);
        return *it;
    }

    Node* Graph::PopBack()
    {
        const std::vector<Node*>::iterator& it = nodes.end()-1;
        nodes.erase(it);
        return *it;
    }

    void Graph::Draw() const
    {
        glPointSize(3);
        FOR_EACH(it, nodes){
            (*it)->Draw();
        }
    }

    bool Graph::GoodDirection(const Pose& pose, meters_t range, radians_t& heading_result)
    {
        if(nodes.empty())
            return 0; // a null guess

        Node* best_node = NULL;

        // find the closest node
        FOR_EACH(it, nodes){
            Node* node = *it;
            double dist = hypot(node->pose.x-pose.x, node->pose.y-pose.y);

            // if it's in range, and either its the first we have found,
            // or it has a lower value than the current best
            if(dist < range && (best_node == NULL || node->value < best_node->value)){
                best_node = node;
            }
        }

        if(best_node == NULL){
            printf("[%s:%d] [robot %d]: path planner: no nodes in range\n", StripPath(__FILE__), __LINE__, robot);
            return false;
        }

        //else
        heading_result = normalize(atan2(best_node->pose.y-pose.y, best_node->pose.x-pose.x));

        return true;
    }

    unsigned int Graph::Size()
    {
        return nodes.size();
    }

    GraphVis::GraphVis(Graph** graphpp) : Visualizer("graph", "vis_graph")
    {
        this->graphpp = graphpp;
    }

    GraphVis::~GraphVis(){

    }

    void GraphVis::Visualize(Model* mod, Camera* cam)
    {
        if(*graphpp == NULL)
            return;

        glPushMatrix();

        Gl::pose_inverse_shift(mod->GetGlobalPose());

        Color c = mod->GetColor();
        c.a = 0.4;

        mod->PushColor(c);
        (*graphpp)->Draw();
        mod->PopColor();

        glPopMatrix();
    }
}
