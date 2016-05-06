#ifndef EAE_H
#define EAE_H

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
 * Namespace for energy aware exploration (eae).
 */
namespace eae
{
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
     * The current state of a robot.
     */
    typedef enum{
        STATE_IDLE = 0,
        STATE_EXPLORE,
        STATE_PRECHARGE,
        STATE_CHARGE,
        STATE_DEAD,
        STATE_FINISHED
    } robot_state_t;

    /**
     * The current state of a docking station.
     */
    typedef enum{
        STATE_VACANT = 0,
        STATE_OCCUPIED
    } ds_state_t;

    /**
     * Wifi message types.
     */
    typedef enum{
        MSG_ROBOT = 0,
        MSG_DS,
        MSG_FRONTIER_AUCTION,
        MSG_DS_AUCTION
    } msg_type_t;

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
     * A class for coordinating the robots using auctions.
     */
    class Coordination
    {
    public:
        /**
         * Constructor
         */
        Coordination();

        /**
         * Initiate an auction for distributing frontiers amongst robots.
         *
         * @param Pose* frontier: The position of the frontier.
         * @param double bid: The bid for the auction.
         */
        bool FrontierAuction(Pose* frontier, double bid);

    private:
        /**
         * The wifi model of the robot.
         */
        ModelWifi* wifi;

        /**
         * The position model of the robot.
         */
        ModelPosition* pos;

        /**
         * Callback function that is called when @todo
         *
         * @param ModelWifi* wifi: The instantiated wifi model of the robot.
         * @param Robot* robot: The instantiated robot object which attached the callback.
         *
         * @return int: Returns 0.
         */
        static int WifiUpdate(ModelWifi* wifi, Robot* robot);

        /**
         * @todo
         */
        static void ProcessMessage(WifiMessageBase* incoming);
    };

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
        GridMap();

        /**
         * Visualize the map in the command line.
         */
        void Visualize();

        /**
         * Insert a value into the map.
         * If the specified coordinates exceed the map dimension, the map will be extended.
         *
         * @param int x: X-coordinate of the cell.
         * @param int y: Y-coordinate of the cell.
         * @param char val: Value to insert.
         */
        void Insert(int x, int y, char val);

        /**
         * Mark all cells visible to the robot at cell (x,y) as visited.
         *
         * @param int x: X-coordinate of the cell.
         * @param int y: Y-coordinate of the cell.
         *
         * @todo: Don't always mark as free, but according to actual sensor reading.
         * @todo: Don't just mark direct neighbors but according to actual sensor range.
         */
        void Clear(int x, int y);

        /**
         * Get a list of all frontiers.
         *
         * @return vector< vector <int> >: A vector containing an element for every frontier. Each element ist a vector with the two coordinates of that frontier.
         */
        vector< vector <int> > Frontiers();

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
        char Read(int x, int y);

        /**
         * Write a value to a grid cell.
         * The given coordinates will be converted to the correct indices.
         * Out of range exceptions are not handled!
         *
         * @param int x: X-coordinate of the cell.
         * @param int y: Y-coordinate of the cell.
         * @param char val: The value of the grid cell.
         */
        void Write(int x, int y, char val);

        /**
         * Determines if a cell is a frontier cell.
         *
         * @param int x: X-coordinate of the cell.
         * @param int y: Y-coordinate of the cell.
         *
         * @return bool: True, if it is a frontier cell, false otherwise.
         */
        bool Frontier(int x, int y);
    };

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
        LogOutput();

        /**
         * Destructor
         * Closes the log file.
         */
        ~LogOutput();

        /**
         * Write a string as one line to the log file.
         */
        void Write(string text);

    private:
        /**
         * The file stream for the log file.
         */
        ofstream file;
    };

    /**
     * A class that defines the behavior of a robot.
     */
    class Robot
    {
    public:
        /**
         * Constructor
         *
         * @param ModelPosition* pos: The instantiated position model of the robot.
         */
        Robot(ModelPosition* pos);

        /**
         * Exploration routine, drives the robots towards unknown space.
         */
        void Explore();

    private:
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
        double Distance(double from_x, double from_y, double to_x, double to_y);

        /**
         * Compute the distance from the current location to a point.
         *
         * @param double to_x: X-coordinate of end point.
         * @param double to_y: Y-coordinate of end point.
         *
         * @return double: The distance.
         */
        double Distance(double to_x, double to_y);

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
        double Angle(double from_x, double from_y, double to_x, double to_y);

        /**
         * Callback function that is called when the robot changes position.
         * When the robot reached it's goal, it directs the robot to continue exploration.
         *
         * @param ModelPosition* pos: The instantiated position model of the robot.
         * @param Robot* robot: The instantiated robot object which attached the callback.
         *
         * @return int: Returns 0.
         */
        static int PositionUpdate(ModelPosition* pos, Robot* robot);

        /**
         * GridMap object for internal representation of the map.
         */
        GridMap* map;

        /**
         * LogOutput object for logging of data to log files.
         */
        LogOutput* log;

        /**
         * The position model of the robot.
         */
        ModelPosition* pos;

        /**
         * The coordination object.
         */
        Coordination* cord;

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
         * The current state of the robot.
         */
        robot_state_t state;

        /**
         * The total distance the robot traveled.
         */
        double dist_travel;
    };

    /**
     * A class for the wifi messages.
     *
     * @todo: implement
     */
    class WifiMessage : public WifiMessageBase
    {
    public:
        /**
         * Constructor
         */
        WifiMessage();

        /**
         * Destructor
         */
        ~WifiMessage();

        /**
         * Copy constructor
         *
         * @param WifiMessage& toCopy: The object that should be copied.
         */
        WifiMessage(const WifiMessage& toCopy);

        /**
         * Equals operator
         *
         * @param WifiMessage& toCopy: The object that should be copied.
         */
        WifiMessage& operator=(const WifiMessage& toCopy);

        /**
         * Clones a message object.
         */
        virtual WifiMessage* Clone();
    private:
        msg_type_t type;
    };

    /**
     * A class for wifi messages containing robot information.
     */
    class WifiMessageRobot : WifiMessage
    {
    private:
        int id;
        robot_state_t state;
    };

    /**
     * A class for wifi messages containing docking station information.
     */
    class WifiMessageDs : WifiMessage
    {
    private:
        int id;
        ds_state_t state;
        Pose pos;
    };

    /**
     * A class for wifi messages for auctioning of docking stations.
     */
    class WifiMessageFrontierAuction : WifiMessage
    {
    private:
        int auction;
        int robot;
        Pose frontier;
        double bid;
    };

    /**
     * A class for wifi messages for auctioning of docking stations.
     */
    class WifiMessageDsAuction : WifiMessage
    {
    private:
        int auction;
        int robot;
        int ds;
        double bid;
    };
}

#endif
