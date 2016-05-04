#include "stage.hh"
#include <fstream>
#include <ctime>
#include <string>
#include <stdexcept>
#include <sstream>
#include <sys/stat.h>
//#include <boost/filesystem.hpp>

using namespace Stg;
using namespace std;

/**
 * Power consumption in watts per (meter per second) per kg.
 * Copied from libstage/model_position.cc line 88.
 */
static const double WATTS_KGMS = 10.0;

/**
 * Power consumption in watts when robot is stationary.
 * Copied from libstage/model_position.cc line 89.
 */
static const double WATTS = 1.0;

/**
 * Distance that the robot can be away from goal.
 */
const double goal_tolerance = 0.05;

/**
 * Offset of the map array.
 */
const int map_offset = 50;

/**
 * Math constant π.
 */
const double pi = 3.14159;

/**
 * Constants for map grid cell values.
 */
const char occ = 2  ; // occupied grid cell (obstacle)
const char unk = 1;   // unknown grid cell
const char fre = 0;   // free grid cell

/**
 * Weights for the cost function.
 */
const int w1 = 10;
const int w2 = 0;
const int w3 = 9;
const int w4 = 20;

/**
 * The current mode of the robot.
 */
typedef enum {
  MODE_IDLE=0,
  MODE_EXPLORE,
  MODE_CHARGE,
  MODE_DEAD,
  MODE_FINISHED
} nav_mode_t;

/**
 * Path for log files. Subdirectories are automatically created.
 */
const string log_path = "/media/mrappapo/Daten/Arbeit/NES/projects/2014_mrs/simulation/cpp_dock-coord/";

/**
 * Map array definitions.
 */
//typedef boost::multi_array<char, 2> grid_t; // grid map type
//typedef grid_t::index idx;                   // index
//const int dim = 25;                         // initial dimension of map


/**
 * A class that defines the behavior of a robot.
 */
class Robot
{
private:

    /*class MapVis : public Visualizer
    {
    public:
        MapVis() : Visualizer( "Map", "vis_map" ) {}
        virtual ~MapVis() {}

        virtual void Visualize( Model* mod, Camera* cam )
        {
            Gl::draw_vector(1.25, 0, 0);
        }
    };*/

    /**
     * A class for the internal representation of the map.
     */
    class GridMap
    {
    public:
        /**
         * Constructor
         *
         * @todo: Fill map and set dimension parameters based on laser range.
         */
        GridMap()
        {
            // set dimension parameters
            x_dim = 3;
            y_dim = 3;
            x_offset = 1;
            y_offset = 1;
            x_min = 0-x_offset;
            x_max = 0+x_offset;
            y_min = 0-y_offset;
            y_max = 0+y_offset;

            // fill grid map with unknown values
            vector<char> col(y_dim, unk);
            for(int i=0; i<x_dim; ++i){
                grid.push_back(col);
            }
        }

        /**
         * Visualize the map in the command line.
         */
        void visualize()
        {
            for(unsigned int i=0; i<grid.at(0).size(); ++i){
                for(it = grid.begin(); it<grid.end(); ++it){
                    printf("%d ", it->at(i));
                }
                cout << endl;
            }
            cout << endl;
        }

        /**
         * Insert a value into the map.
         * If the specified coordinates exceed the map dimension, the map will be extended.
         *
         * @param int x: X-coordinate of the cell.
         * @param int y: Y-coordinate of the cell.
         * @param char val: Value to insert.
         */
        void insert(int x, int y, char val)
        {
            // x index out of bounds, extend vector in x dimension
            if(x < x_min){
                // number of columns to add
                int add = x_min - x;
                // column containing unknown values
                vector<char> col(y_dim, unk);
                // add column
                for(int i=0; i<add; ++i){
                    grid.insert(grid.begin(), col);
                }
                // adjust dimension parameters
                x_dim += add;
                x_offset += add;
                x_min -= add;
            }

            // x index out of bounds, extend vector in x dimension
            if(x_max < x){
                // number of columns to add
                int add = x - x_max;
                // column containing unknown values
                vector<char> col(y_dim, unk);
                // add column
                for(int i=0; i<add; ++i){
                    grid.push_back(col);
                }
                // adjust dimension parameters
                x_dim += add;
                x_max += add;
            }

            // y index out of bounds, extend vector in y dimension
            if(y < y_min){
                // number of elements to add to both sides of each column
                int add = y_min - y;
                // add elements to every column
                for(it=grid.begin(); it<grid.end(); ++it){
                    for(int i=0; i<add; ++i){
                        it->insert(it->begin(), unk);
                    }
                }
                // adjust dimension parameters
                y_dim += add;
                y_offset += add;
                y_min -= add;
            }

            // y index out of bounds, extend vector in y dimension
            if(y_max < y){
                // number of elements to add to both sides of each column
                int add = y - y_max;
                // add elements to every column
                for(it=grid.begin(); it<grid.end(); ++it){
                    for(int i=0; i<add; ++i){
                        it->push_back(unk);
                    }
                }
                // adjust dimension parameters
                y_dim += add;
                y_max += add;
            }

            // insert value
            try{
                write(x, y, val);
            }
            catch(const out_of_range& e){
                printf("Could not write '%d' to (%d,%d)! Something went wrong when extending the map!\n", val, x, y);
            }
        }

        /**
         * Mark all cells visible to the robot at cell (x,y) as visited.
         *
         * @param int x: X-coordinate of the cell.
         * @param int y: Y-coordinate of the cell.
         *
         * @todo: Don't always mark as free, but according to actual sensor reading.
         * @todo: Don't just mark direct neighbors but according to actual sensor range.
         */
        void clear(int x, int y)
        {
            insert(x-1, y-1, fre);
            insert(x-1, y, fre);
            insert(x-1, y+1, fre);
            insert(x, y-1, fre);
            insert(x, y, fre);
            insert(x, y+1, fre);
            insert(x+1, y-1, fre);
            insert(x+1, y, fre);
            insert(x+1, y+1, fre);
        }

        /**
         * Get a list of all frontiers.
         *
         * @return vector< vector <int> >: A vector containing an element for every frontier. Each element ist a vector with the two coordinates of that frontier.
         */
        vector< vector <int> > frontiers()
        {
            vector< vector <int> > frontiers;

            int x = x_min;

            for(it = grid.begin(); it<grid.end(); ++it){
                int y = y_min;
                for(jt = it->begin(); jt<it->end(); ++jt){
                    if(frontier(x,y)){
                        vector<int> frontier;
                        frontier.push_back(x);
                        frontier.push_back(y);
                        frontiers.push_back(frontier);
                    }
                    ++y;
                }
                ++x;
            }

            return frontiers;
        }

    private:
        /**
         * Variable holding the grid map.
         */
        //grid_t grid(boost::extents[dim][dim]);
        //grid_t grid;
        vector< vector <char> > grid;

        /**
         * Variables for the map dimensions.
         */
        int x_dim;
        int y_dim;
        int x_offset;
        int y_offset;
        int x_min;
        int x_max;
        int y_min;
        int y_max;

        /**
         * Map iterators.
         */
        vector< vector<char> >::iterator it;
        vector<char>::iterator jt;

        /**
         * Read the value of one grid cell.
         * The given coordinates will be converted to the correct indices.
         * Out of range exceptions are not handled!
         *
         * @param int x: X-coordinate of the cell.
         * @param int y: Y-coordinate of the cell.
         *
         * @return char: The value of the grid cell.
         */
        char read(int x, int y)
        {
            return grid.at(x+x_offset).at(y+y_offset);
        }

        /**
         * Write a value to a grid cell.
         * The given coordinates will be converted to the correct indices.
         * Out of range exceptions are not handled!
         *
         * @param int x: X-coordinate of the cell.
         * @param int y: Y-coordinate of the cell.
         * @param char val: The value of the grid cell.
         */
        void write(int x, int y, char val)
        {
            grid.at(x+x_offset).at(y+y_offset) = val;
        }

        /**
         * Determines if a cell is a frontier cell.
         *
         * @param int x: X-coordinate of the cell.
         * @param int y: Y-coordinate of the cell.
         *
         * @return bool: True, if it is a frontier cell, false otherwise.
         */
        bool frontier(int x, int y)
        {
            // check cell itself
            try{
                // cell is not free, cannot be frontier
                if(read(x,y) != fre){
                    //printf("cell (%d,%d) is not free!\n", x, y);
                    return false;
                }
            }
            catch(const out_of_range& e){
                // cell is unknown, cannot be frontier
                //printf("cell (%d,%d) out of range!\n", x, y);
                return false;
            }

            // check neighboring cells
            try{
                // all neighbors are known, cannot be frontier
                if(read(x-1,y-1) != unk && read(x-1,y) != unk && read(x-1,y+1) != unk && read(x,y-1) != unk && read(x,y+1) != unk && read(x+1,y-1) != unk && read(x+1,y) != unk && read(x+1,y+1) != unk){
                    //printf("neighbors of cell (%d,%d) are all NOT unknown!\n", x, y);
                    return false;
                }
                // one of the neighboring cells is unknown, it is a frontier
                //printf("at least one neighbor of cell(%d,%d) is unknown!\n", x, y);
                return true;
            }
            catch(const out_of_range& e){
                // one of the neighboring cells is unknown, it is a frontier
                //printf("neighbor of cell (%d,%d) out of range!\n", x, y);
                return true;
            }
        }
    };
    GridMap* map;

    /**
     * A class for logging data.
     */
    class LogOutput
    {
    public:
        /**
         * Constructor
         * Creates a subdirectory in the log_path directory, according to current date.
         */
        LogOutput()
        {
            // get current time
            std::time_t now = time(NULL);
            struct tm* timeinfo;
            timeinfo = localtime(&now);

            // path to log file
            char dir[11];
            strftime(dir, 11, "%y-%m-%d/", timeinfo);
            string path = log_path + string(dir);

            // make directory if it doesn't exist
            mkdir(path.c_str(), 0777);

            // file name
            char filename[7];
            strftime(filename, 7, "%H-%M", timeinfo);

            // complete path of file
            string filepath = path + string(filename) + ".log";
            cout << "log file: " << filepath << endl;

            // open log file
            file.open(filepath.c_str());

            // write header
            write("x position\ty position\tangle\tdistance");
        }

        /**
         * Destructor
         * Closes the log file.
         */
        ~LogOutput()
        {
            file.close();
        }

        /**
         * Write a string as one line to the log file.
         */
        void write(string text)
        {
            file << text << endl;
        }

    private:
        /**
         * The file stream for the log file.
         */
        ofstream file;
    };
    LogOutput* log;

    /**
     * A class for the wifi messages.
     *
     * @todo: implement
     */
    class MyWifiMessage : public WifiMessageBase
    {
    public:
        MyWifiMessage():WifiMessageBase(){};

        ~MyWifiMessage(){ };

        Pose gpose;

        MyWifiMessage(const MyWifiMessage& toCopy) : WifiMessageBase(toCopy)
        {
            gpose = toCopy.gpose;
        };

        MyWifiMessage& operator=(const MyWifiMessage& toCopy)
        {
            WifiMessageBase::operator=(toCopy);
            gpose = toCopy.gpose;
            return *this;
        };

        virtual WifiMessageBase* Clone()
        {
            return new MyWifiMessage(*this);
        };
    };

    /**
     * The position model of the robot.
     */
    ModelPosition* pos;

    /**
     * The wifi model of the robot.
     */
    ModelWifi* wifi;

    /**
     * The camera object that is used for visualization.
     */
    OrthoCamera* cam;

    /**
     * The goal where the robot navigates to.
     */
    Pose goal;

    /**
     * The previous goal/location of the robot.
     */
    Pose prev_pose;

    /**
     * The color for visualizing the waypoints.
     */
    Color wpcolor;

    /**
     * The current mode of the robot.
     */
    nav_mode_t mode;

    /**
     * The total distance the robot traveled.
     */
    double dist_travel;

    /**
     * Compute distance between two points.
     *
     * @param double from_x: X-coordinate of starting point.
     * @param double from_y: Y-coordinate of starting point.
     * @param double to_x: X-coordinate of end point.
     * @param double to_y: Y-coordinate of end point.
     *
     * @return double: The distance.
     */
    double Distance(double from_x, double from_y, double to_x, double to_y)
    {
        double x = to_x - from_x;
        double y = to_y - from_y;
        return hypot(x,y);
    }

    /**
     * Compute the distance from the current location to a point.
     *
     * @param double to_x: X-coordinate of end point.
     * @param double to_y: Y-coordinate of end point.
     *
     * @return double: The distance.
     */
    double Distance(double to_x, double to_y)
    {
        return Distance(pos->GetPose().x, pos->GetPose().y, to_x, to_y);
    }

    /**
     * Compute the angle between two points starting from the positive x-axis.
     *
     * @param double from_x: X-coordinate of starting point.
     * @param double from_y: Y-coordinate of starting point.
     * @param double to_x: X-coordinate of end point.
     * @param double to_y: Y-coordinate of end point.
     *
     * @return double: The angle in radian.
     */
    double Angle(double from_x, double from_y, double to_x, double to_y)
    {
        double x = to_x - from_x;
        double y = to_y - from_y;
        return atan2(y,x);
    }

    /**
     * Callback function that is called when the robot changes position.
     * When the robot reached it's goal, it directs the robot to continue exploration.
     *
     * @param ModelPosition* pos: The instantiated position model of the robot.
     * @param Robot* robot: The instantiated robot object which attached the callback.
     *
     * @return int: Returns 0.
     */
    static int PositionUpdate(ModelPosition* pos, Robot* robot)
    {
        // robot reached goal, continue exploration
        if(pos->GetPose().Distance(robot->goal) < goal_tolerance && (robot->mode == MODE_EXPLORE || robot->mode == MODE_IDLE)){
            robot->Explore();
        }
        //printf("velocity:  x=%.1f, y=%.1f, z=%.1f, a=%.1f\n", pos->GetVelocity().x, pos->GetVelocity().y, pos->GetVelocity().z, pos->GetVelocity().a);

        return 0; // run again
    }

    /**
     * Callback function that is called when @todo
     *
     * @param ModelWifi* wifi: The instantiated wifi model of the robot.
     * @param Robot* robot: The instantiated robot object which attached the callback.
     *
     * @return int: Returns 0.
     */
    static int WifiUpdate(ModelWifi* wifi, Robot* robot)
    {
        // visualize wifi connections
        wifi->DataVisualize(robot->cam);

        // broadcast a test message
        MyWifiMessage msg;
        msg.gpose = robot->pos->GetGlobalPose();
        WifiMessageBase* base_ptr = &msg;
        wifi->comm.SendBroadcastMessage(base_ptr);

        return 0; // run again
    }

    /**
     * @todo
     */
    static void ProcessMessage(WifiMessageBase* incoming)
    {
        //printf("processing message\n");
        MyWifiMessage * my_mesg = dynamic_cast<MyWifiMessage*>(incoming);
        if ( my_mesg )
        {
            printf("Robot [%u]: Neighbor [%u] is at (%.2f %.2f) and heading (%.2f)\n",
            my_mesg->GetRecipientId(), my_mesg->GetSenderId(), my_mesg->gpose.x, my_mesg->gpose.y, my_mesg->gpose.a );
        }
        delete incoming;
    }

public:
    /**
     * Constructor
     *
     * @param ModelPosition* pos: The instantiated position model of the robot.
     */
    Robot(ModelPosition* pos) : map(), pos(pos), cam(), prev_pose(pos->GetPose()), wpcolor(0,1,0), mode(MODE_IDLE)//, mapvis()
    {
        cout << endl;

        // create new map
        map = new GridMap();

        // create log file
        log = new LogOutput();

        // register callback for position updates
        pos->AddCallback(Model::CB_UPDATE, (model_callback_t)PositionUpdate, this );
        pos->Subscribe();

        // initialize wifi adapter
        wifi = (ModelWifi*)pos->GetChild("wifi:0");
        wifi->AddCallback(Model::CB_UPDATE, (model_callback_t)WifiUpdate, this);
        wifi->comm.SetReceiveMsgFn(ProcessMessage);
        wifi->Subscribe();

        // store starting point for visualization
        pos->waypoints.push_back(ModelPosition::Waypoint(0, 0, 0, 0, wpcolor));

        /*World* world_map = pos->GetWorld();
        Model* ground = world_map->GetGround();
        pos->AddVisualizer(&mapvis, true);
        mapvis.Visualize(ground, cam);*/

        // distance traveled
        dist_travel = 0;
    }

    /**
     * Exploration routine, drives the robots towards unknown space.
     */
    void Explore()
    {
        // only explore if currently exploring or idle
        if(mode != MODE_EXPLORE){
            if(mode != MODE_IDLE)
                return;
            mode = MODE_EXPLORE;
        }


        /**************
         * update map *
         **************/

        // get current position
        Pose pose = pos->GetPose();

        // mark neighborhood as visited
        map->clear(round(pose.x), round(pose.y));
        //map->visualize();


        /*******************************
         * estimate remaining distance *
         *******************************/

        double velocity; // average velocity
        double power;    // power consumption at average velocity
        double dist_rem; // remaining distance that the robot can drive with its current battery
        World* world = pos->GetWorld();

        // calculate average velocity
        if(dist_travel > 0 && world->SimTimeNow() > 0){
            velocity = dist_travel/world->SimTimeNow()*1000000;
        }
        // at beginning of simulation take max velocity
        else{
            velocity = pos->velocity_bounds->max;
        }

        // calculate power consumption (according to stage model: libstage/model_position:496)
        power = velocity * WATTS_KGMS * pos->GetTotalMass() + WATTS;

        // calculate remaining distance
        dist_rem = pos->FindPowerPack()->GetStored() / power * velocity;

        //printf("power: %.1f, dist: %.1f\n", power, dist_rem);


        /******************
         * determine goal *
         ******************/

        // initialize goal with current position
        goal = pose;

        // parameters for cost function
        double dg;    // distance to frontier
        double dgb;   // distance from frontier to home base
        double dgbe;  // energy-aware distance from frontier to home base
        double theta; // required turning of robot to reach frontier
        double cost;  // total cost
        double min_cost = 200; // initialize cost

        // frontier location
        int x,y;

        // get list of frontiers
        vector< vector <int> > frontiers = map->frontiers();

        // iterate through all frontiers
        vector< vector<int> >::iterator it;
        for(it=frontiers.begin(); it<frontiers.end(); ++it){
            // frontier location
            x = it->at(0);
            y = it->at(1);

            // compute distances
            dg = Distance(x, y);
            dgb = Distance(x, y, 0, 0);

            // not enough energy to reach frontier
            if(dist_rem <= dg + dgb)
                continue;

            // energy-aware distance
            if(pos->FindPowerPack()->ProportionRemaining() > 0.5)
                dgbe = -dgb;
            else
                dgbe = dgb;

            // compute required turning
            theta = 1/pi * (pi - abs(abs(Angle(prev_pose.x, prev_pose.y, pose.x, pose.y) - Angle(pose.x, pose.y, x, y)) - pi));

            // compute total cost
            cost = w1*dg + w2*dgb + w3*dgbe + w4*theta;

            // minimize cost
            if(cost < min_cost){

                /*printf("coordinate:\n");
                printf("x: %d\n", x);
                printf("y: %d\n\n", y);
                printf("cost function:\n");
                printf("dg:    %.2f\n", dg);
                printf("dgb:   %.2f\n", dgb);
                printf("dgbe:  %.2f\n", dgbe);
                printf("theta: %.2f\n", theta);
                printf("\n");*/

                min_cost = cost;
                goal.x = x;
                goal.y = y;
                goal.a = Angle(pose.x, pose.y, x, y);
            }
        }

        // no new goal was found
        if(goal == pose){

            // end of exploration
            if(Distance(0, 0) < goal_tolerance){
                mode = MODE_FINISHED;
                printf("exploration finished!\n");
                return;
            }

            // go recharging
            else{
                goal.x = 0;
                goal.y = 0;
                goal.a = Angle(pose.x, pose.y, 0, 0);
            }
        }


        /****************
         * move to goal *
         ****************/

        // store goal for visualization
        pos->waypoints.push_back(ModelPosition::Waypoint(goal, wpcolor));

        // store traveled distance
        dist_travel += pose.Distance(goal);

        // store previous position
        prev_pose = pose;

        // move robot
        pos->GoTo(goal);

        stringstream output;
        output << goal.x << "\t" << goal.y << "\t" << goal.a << "\t" << dist_travel;
        log->write(output.str());
        //printf("move to (%.0f,%.0f,%.1f), cost %.1f\n\n\n", goal.x, goal.y, goal.a, min_cost);
    }

};

/**
 * Initialization function that is called by Stage when the model starts up.
 */
extern "C" int Init(Model* mod, CtrlArgs* args)
{
    Robot* robot = new Robot((ModelPosition*)mod);

    robot->Explore();

    return 0; // ok
}
