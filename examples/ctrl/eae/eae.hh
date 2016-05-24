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
     * Distance that the robot can be away from goal.
     */
    const double GOAL_TOLERANCE = 0.05;

    /**
     * Math constant π.
     */
    const double PI = 3.14159265358979323846264338327950288419716939937510;

    /**
     * Value for an invalid bid.
     */
    const double BID_INV = 0;

    /**
     * Value for initial bid.
     */
    const double BID_INIT = -200;

    /**
     * Map grid cell values.
     */
    typedef enum{
        CELL_FREE = 0,
        CELL_UNKNOWN,
        CELL_OCCUPIED
    } grid_cell_t;

    /**
     * The current state of a robot.
     */
    typedef enum{
        STATE_UNDEFINED_ROBOT = 0,
        STATE_IDLE,
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
        STATE_UNDEFINED_DS = 0,
        STATE_VACANT,
        STATE_OCCUPIED
    } ds_state_t;

    /**
     * Wifi message types.
     */
    typedef enum{
        MSG_UNDEFINED = 0,
        MSG_ROBOT,
        MSG_DS,
        MSG_FRONTIER_AUCTION,
        MSG_DS_AUCTION,
        MSG_MAP
    } msg_type_t;

    /**
     * Forward class declarations.
     */
    class Coordination;
    class LogOutput;
    class Robot;
    class WifiMessage;
    class WifiMessageRobot;
    class WifiMessageDs;
    class WifiMessageFrontierAuction;
    class WifiMessageDsAuction;
}

#endif
