#ifndef SWARM_H
#define SWARM_H

#include "stage.hh"
#include "worldfile.hh"
#include "../astar/astar.h"
#include <fstream>
#include <ctime>
#include <string>
#include <stdexcept>
#include <sstream>
#include <sys/stat.h>

using namespace Stg;
using namespace std;

/**
 * @brief Namespace for swarm based exploration.
 */
namespace swarm
{
    /**
     * Display additional debugging output for the robots listed in this array.
     * Only debug output of the robots with the given ID's will be output.
     * Robots can have following ID's: 1, 6, 11, 16, 21, 26, 31, 36, ...
     */
    //const int DEBUG_ROBOTS[] = {1, 6, 11, 16, 21, 26, 31, 36}; // debug all
    const int DEBUG_ROBOTS[] = {}; // debug none

    /**
     * Distance that the robot can be away from goal.
     */
    const double GOAL_TOLERANCE = 0.1;

    /**
     * Math constant π.
     */
    const double PI = 3.14159265358979323846264338327950288419716939937510;

    /**
     * Range of laser scanner in grid cells.
     */
    const int LASER_RANGE = 10;

    /**
     * Map grid cell values.
     */
    typedef enum{
        CELL_FREE = 0,
        CELL_UNKNOWN,
        CELL_OCCUPIED
    } grid_cell_v;

    /**
     * Map grid cell coordinate type.
     */
    typedef struct{
        int x;
        int y;
    } grid_cell_t;

    /**
     * The current state of a robot.
     */
    typedef enum{
        STATE_UNDEFINED_ROBOT = 0,
        STATE_INIT,
        STATE_IDLE,
        STATE_EXPLORE,
        STATE_CHARGE_QUEUE,
        STATE_PRECHARGE,
        STATE_GOING_CHARGING,
        STATE_CHARGE,
        STATE_DEAD,
        STATE_FINISHED
    } robot_state_t;

    /**
     * Strings describing the robot state.
     * Make sure they match the robot_state_t enum!
     */
    const string STATE_STRING[] = {"undefined", "init", "idle", "explore", "charge queue", "precharge", "going charging", "charge", "dead", "finished"};

    /**
     * Wifi message types.
     */
    typedef enum{
        MSG_UNDEFINED = 0,
        MSG_ROBOT,
        MSG_DS
    } msg_type_t;

    /**
     * Forward class declarations.
     */
    class Coordination;
    class Edge;
    class Node;
    class Graph;
    class GraphVis;
    class GridMap;
    class LogOutput;
    class Robot;
    class WifiMessage;
    class WifiMessageRobot;
    class WifiMessageDs;

    /**
     * Remove the path from a file name.
     *
     * @param string file: The complete file name including path.
     *
     * @return const char*: The file name without path.
     */
    inline const char* StripPath(string file)
    {
        return file.substr(file.find_last_of("/")+1).c_str();
    }

    /**
     * Check if a value exists in an array.
     * The type of the array values is the template parameter T.
     *
     * @param T val: The value to check.
     * @param T arr[]: The array too look in.
     * @param int size: The size of the array (number of elements).
     *
     * @return bool: True if the value exists in the array, false otherwise.
     */
    template<typename T>
    inline const bool InArray(T val, const T arr[], int size)
    {
        // loop through array and look vor val
        for(int i=0; i<size; ++i){
            // value found, return true
            if(arr[i] == val)
                return true;
        }

        // value not found
        return false;
    }
}

#endif
