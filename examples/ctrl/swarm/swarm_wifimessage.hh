#ifndef SWARM_WIFIMESSAGE_H
#define SWARM_WIFIMESSAGE_H

#include "swarm.hh"
#include "swarm_ds.hh"
#include "swarm_gridmap.hh"

using namespace Stg;
using namespace std;

/**
 * Namespace for energy aware exploration (swarm).
 */
namespace swarm
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
         * @param int id_robot: ID of the robot.
         * @param robot_state_t state_robot: State of the robot.
         * @param int id_ds: ID of the docking station.
         * @param Pose pos: Position of robot, docking station, or frontier.
         */
        WifiMessage(msg_type_t type, int id_robot, robot_state_t state_robot, int id_ds, Pose pos);

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
         * ID of the robot.
         */
        int id_robot;

        /**
         * ID of the docking station.
         */
        int id_ds;

        /**
         * Position of robot, docking station, or frontier, depending on the subclass used.
         */
        Pose pose;
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
         * @param Pose pose: Position of docking station.
         */
        WifiMessageDs(int id_ds, Pose pose);
    };
}

#endif
