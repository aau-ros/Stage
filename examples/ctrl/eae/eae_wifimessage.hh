#ifndef EAE_WIFIMESSAGE_H
#define EAE_WIFIMESSAGE_H

#include "eae.hh"
#include "eae_ds.hh"
#include "eae_gridmap.hh"

using namespace Stg;
using namespace std;

/**
 * Namespace for energy aware exploration (eae).
 */
namespace eae
{
    /**
     * A class for the wifi messages.
     * The data fields are used depending on the subclass that is used.
     */
    class WifiMessage : public WifiMessageBase
    {
    public:
        /**
         * Constructor that sets default values.
         */
        WifiMessage();

        /**
         * Constructor that takes parameters to set data fields.
         *
         * @param msg_type_t type: Message type.
         * @param int id_auction: ID of the auction.
         * @param int id_robot: ID of the robot.
         * @param robot_state_t state_robot: State of the robot.
         * @param int id_ds: ID of the docking station.
         * @param ds_state_t state_ds: State of the docking station.
         * @param double bid: Bid for the auction.
         * @param Pose pos: Position of robot, docking station, or frontier.
         * @param int change: Change in number of robots.
         */
        WifiMessage(msg_type_t type, int id_auction, int id_robot, robot_state_t state_robot, int id_ds, ds_state_t state_ds, double bid, Pose pos, int change);

        /**
         * Constructor for map messages.
         *
         * @param GridMap* map: The grid map.
         */
        WifiMessage(GridMap* map);

        /**
         * Copy constructor.
         *
         * @param WifiMessage& toCopy: The object that should be copied.
         */
        WifiMessage(const WifiMessage& toCopy);

        /**
         * Destructor.
         */
        ~WifiMessage();

        /**
         * Assignment operator.
         *
         * @param WifiMessage& toCopy: The object that should be copied.
         *
         * @return WifiMessage&: A pointer to this object.
         */
        WifiMessage& operator=(const WifiMessage& toCopy);

        /**
         * Clones a message object.
         */
        virtual WifiMessage* Clone();

        /**
         * Create a string of all data fields.
         *
         * @return string: The string containing all data fields.
         */
        string ToString();

        /**
         * Message type.
         */
        msg_type_t type;

        /**
         * State of the robot.
         */
        robot_state_t state_robot;

        /**
         * State of the docking station.
         */
        ds_state_t state_ds;

        /**
         * ID of the auction.
         */
        int id_auction;

        /**
         * ID of the robot.
         */
        int id_robot;

        /**
         * ID of the docking station.
         */
        int id_ds;

        /**
         * Bid for the auction.
         */
        double bid;

        /**
         * Position of robot, docking station, or frontier, depending on the subclass used.
         */
        Pose pose;

        /**
         * Map.
         */
        GridMap* map;

        /**
         * Change in number of robots that selected a docking station.
         */
        int change;
    };

    /**
     * A class for wifi messages containing robot information.
     */
    class WifiMessageRobot : public WifiMessage
    {
    public:
        /**
         * Constructor.
         *
         * @param int id_robot: ID of the robot.
         * @param robot_state_t state_robot: State of the robot.
         * @param Pose pose: Position of robot.
         */
        WifiMessageRobot(int id_robot, robot_state_t state_robot, Pose pose);
    };

    /**
     * A class for wifi messages containing docking station information.
     */
    class WifiMessageDs : public WifiMessage
    {
    public:
        /**
         * Constructor.
         *
         * @param int id_ds: ID of the docking station.
         * @param ds_state_t state_ds: State of the docking station.
         * @param Pose pose: Position of docking station.
         * @param int change: Change in number of robots, default 0.
         */
        WifiMessageDs(int id_ds, ds_state_t state_ds, Pose pose, int change=0);
    };

    /**
     * A class for wifi messages for keeping the order in docking station queues.
     */
    class WifiMessageDsQueue : public WifiMessage
    {
    public:
        /**
         * Constructor.
         *
         * @param int id_to: ID of the robot that this robot is replying to.
         * @param int id_from: ID of the sending robot.
         * @param int id_ds: ID of the docking station.
         */
        WifiMessageDsQueue(int id_to, int id_from, int id_ds);
    };

    /**
     * A class for the map update wifi messages.
     */
    class WifiMessageMap : public WifiMessage
    {
    public:
        /**
         * Constructor.
         *
         * @param GridMap* map: The grid map.
         */
        WifiMessageMap(GridMap* map);
    };
}

#endif
