#ifndef SWARM_COORDINATION_H
#define SWARM_COORDINATION_H

#include "swarm.hh"
#include "swarm_ds.hh"
#include "swarm_robot.hh"
#include "swarm_wifimessage.hh"

using namespace Stg;
using namespace std;

/**
 * Namespace for energy aware exploration (swarm).
 */
namespace swarm
{
    /**
     * Timeout for beacons.
     * It defines the interval at which robot beacons are send.
     * Robots are removed from the local vector when no update has been received in twice the timeout.
     */
    const usec_t TO_BEACON = 1000000;

    /**
     * Tolerance when calculating the distance to the closest docking station.
     */
    const double DS_TOLERANCE = 2;
    
    /**
     * Distance at which robots are considered close by.
     * It is used for computing the robot density in a sector.
     */
    const double CLOSE_DIST = 2;

    /**
     * Robot type.
     */
    typedef struct{
        int id;
        robot_state_t state;
        Pose pose;
        usec_t update;
    } robot_t;

    /**
     * A class for coordinating the robots using auctions.
     */
    class Coordination
    {
    public:
        /**
         * Constructor.
         *
         * @param int robot: Id of robot owning this object.
         */
        Coordination(ModelPosition* pos, Robot* robot);

        /**
         * Update or add a docking station to the private vector.
         *
         * @param int id: The ID of the docking station (as returned by the fiducial).
         * @param Pose pose: The position of the docking station.
         */
        void UpdateDs(int id, Pose pose);

        /**
         * Check if all other robots finished the exploration already.
         *
         * @return bool: True if all other robots are in state STATE_FINISHED.
         */
        bool Finished();

        /**
         * Get the wifi model.
         *
         * @return string: The name of the wifi model.
         */
        string GetWifiModel();
        
        /**
         * Get the total number of robots in range.
         * These are robots from which a beacon has been received in twice the beacon timeout.
         */
        int NumRobots();
        
        /**
         * Compute the robot density in a given sector using positions retrieved over wifi.
         * This includes only robots from which a beacon has been received in twice the beacon timeout.
         * 
         * @param radians_t angle_min: The angle at which the sector starts.
         * @param radians_t angle_max: The angle at which the sector ends.
         * 
         * @return float: The relative number of robots in a sector normalized to their distances.
         */
        float RobotDensity(radians_t angle_min, radians_t angle_max);

        /**
         * Number of messages sent over wifi.
         */
        unsigned int msgs_sent;

        /**
         * Number of messages received over wifi.
         */
        unsigned int msgs_received;

        /**
         * Number of bytes sent over wifi.
         */
        unsigned int bytes_sent;

        /**
         * Number of bytes received over wifi.
         */
        unsigned int bytes_received;

    private:
        /**
         * Update the vector of robots, i.e. add new robot.
         *
         * @param int id: The ID of the robot to update.
         * @param robot_state_t state: The state of the robot.
         * @param Pose pose: The pose of the robot.
         */
        void UpdateRobots(int id, robot_state_t state, Pose pose);
        
        /**
         * Update the vector of robots, i.e. remove timed out robots.
         */
        void UpdateRobots();

        /**
         * Send a beacon message about the robot.
         */
        void BroadcastBeacon();

        /**
         * Get the docking station object from the private vector.
         *
         * @param int id: The ID of the docking station
         *
         * @return Ds*: Pointer to the docking station in the vector.
         */
        Ds* GetDs(int id);

        /**
         * Get the position of a docking station.
         *
         * @param int id: The ID of the docking station.
         *
         * @return Pose: The position of the docking station.
         */
        Pose DsPose(int id);

        /**
         * Callback function that is called when the wifi model is updated (happens frequently).
         *
         * @param ModelWifi* wifi: The instantiated wifi model of the robot.
         * @param Robot* robot: The instantiated robot object which attached the callback.
         *
         * @return int: Returns 0.
         */
        static int WifiUpdate(ModelWifi* wifi, Coordination* cord);

        /**
         * Callback function for incoming messages.
         *
         * @param WifiMessageBase* incoming: Pointer to the incoming message.
         * @param void* coordination: Pointer to the coordination class.
         */
        static void ProcessMessage(WifiMessageBase* incoming, void* coordination);

        /**
         * The wifi model of the robot.
         */
        ModelWifi* wifi;

        /**
         * The position model of the robot.
         */
        ModelPosition* pos;

        /**
         * Pointer to robot owning an object of this class.
         */
        Robot* robot;

        /**
         * Vector containing all robots in range.
         */
        vector<robot_t> robots;

        /**
         * Vector containing all docking stations.
         */
        vector<Ds*> dss;

        /**
         * Timeout for sending beacons. A beacon will only be send, after this timeout expired.
         */
        usec_t to_beacon;
    };
}

#endif
