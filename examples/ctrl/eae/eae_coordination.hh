#ifndef EAE_COORDINATION_H
#define EAE_COORDINATION_H

#include "eae.hh"
#include "eae_robot.hh"
#include "eae_wifimessage.hh"

using namespace Stg;
using namespace std;

/**
 * Namespace for energy aware exploration (eae).
 */
namespace eae
{
    /**
     * Timeout for auctions.
     * When it expires the current highest bidder will win.
     */
    const usec_t TO_AUCTION = 1000000;

    /**
     * Timeout for beacons.
     * It defines the interval at which robot beacons are send.
     */
    const usec_t TO_BEACON = 1000000;

    /**
     * Auction type.
     */
    typedef struct{
        int id;
        double highest_bid;
        int winner;
        usec_t time;
        bool open;
        Pose pose;
    } auction_t;

    /**
     * Robot type.
     */
    typedef struct{
        int id;
        robot_state_t state;
        Pose pose;
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
         * Initiate an auction for distributing frontiers amongst robots.
         *
         * @param Pose frontier: The position of the frontier.
         * @param double bid: The bid for the auction.
         */
        void FrontierAuction(Pose frontier, double bid);

        /**
         * Send a map update to the other robots.
         */
        void BroadcastMap();

        /**
         * Returns the distance from a given pose to the closest robot.
         *
         * @param Pose pose: The pose to calculate the distance from.
         * @return double: The distance to the closest robot.
         */
        double DistRobot(Pose pose);

        /**
         * ID of auction.
         */
        static int auction_id;

    private:
        /**
         * Update the vector of robots.
         *
         * @param int id: The id of the robot to update.
         * @param robot_state_t state: The state of the robot.
         * @param Pose pose: The pose of the robot.
         */
        void UpdateRobots(int id, robot_state_t state, Pose pose);

        /**
         * Update the grid map of the robot.
         *
         * @param GridMap* map: The grid map.
         */
        void UpdateMap(GridMap* map);

        /**
         * Update the information of an auction in the private vector.
         *
         * @param int id: ID of the auction.
         * @param int robot: ID of robot giving a bid.
         * @param double bid: Bid of the robot.
         * @param Pose frontier: Frontier that is at auction.
         */
        void UpdateAuction(int id, int robot, double bid, Pose frontier);

        /**
         * Check all auctions.
         * If this robot is winner of an auction that timed out, take action.
         */
        void CheckAuctions();

        /**
         * Store a new auction in the private vector.
         */
        void StoreNewAuction(int id, double bid, int winner, usec_t time, bool open, Pose pose);

        /**
         * Send a broadcast message for an auction.
         *
         * @param int id: ID of the auction.
         */
        void BroadcastAuction(int id);

        /**
         * Send a beacon message about the robot.
         */
        void BroadcastBeacon();

        /**
         * Callback function that is called when @todo
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
         * Iterator for for robots vector.
         */
        vector<robot_t>::iterator i_r;

        /**
         * Vector containing all auctions this robot participated in.
         */
        vector<auction_t> auctions;

        /**
         * Iterator for auctions vector.
         */
        vector<auction_t>::iterator i_a;

        /**
         * Timeout for sending beacons. A beacon will only be send, after this timeout expired.
         */
        usec_t to_beacon;
    };
}

#endif
