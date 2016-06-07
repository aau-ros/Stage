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
     * Weights for the cost function.
     *
     * @todo: Find optimal weights.
     */
    const int L1 = 1;
    const int L2 = 3;
    const int L3 = 1;
    const int L4 = 1;

    /**
     * Timeout for auctions.
     * When it expires the current highest bidder will win.
     */
    const usec_t TO_AUCTION = 1000000;

    /**
     * Minimum time between auctions.
     */
    const usec_t TO_NEXT_AUCTION = 2500000;

    /**
     * Timeout for beacons.
     * It defines the interval at which robot beacons are send.
     */
    const usec_t TO_BEACON = 1000000;

    /**
     * Auction type for frontiers.
     */
    typedef struct{
        int id;
        double highest_bid;
        int winner;
        usec_t time;
        bool open;
        Pose pose;
    } fr_auction_t;

    /**
     * Auction type for docking stations.
     */
    typedef struct{
        int id;
        double highest_bid;
        int winner;
        usec_t time;
        bool open;
        int ds_id;
    } ds_auction_t;

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
         * Initiate an auction for assigning robots to docking stations.
         *
         * @param Pose pose: The position of the robot.
         * @return ds_t: The docking station a bid was made for.
         */
        ds_t DockingAuction(Pose pose);

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
         * Add a docking station to the private vector.
         *
         * @param int id: The ID of the docking station (as returned by the fiducial).
         * @param Pose pose: The position of the docking station.
         * @param ds_state_t state: The state of the docking station, default is undefined.
         */
        void AddDs(int id, Pose pose, ds_state_t state=STATE_UNDEFINED_DS);

        /**
         * Add or update a docking station to the private vector.
         *
         * @param int id: The ID of the docking station (as returned by the fiducial).
         * @param double x: The x-coordinate of the docking station.
         * @param double y: The y-coordinate of the docking station.
         * @param double a: Rotation about the z axis.
         */
        void UpdateDs(int id, double x, double y, double a);

        /**
         * Get the docking station that is closest to the robots current location and that is vacant.
         *
         * @param Pose pose: The robots current location.
         * @return ds_t: The docking station.
         */
        ds_t ClosestDs(Pose pose);

        /**
         * Set a docking station to the state vacant.
         *
         * @param int id: The ID of the docking station.
         */
        void DsVacant(int id);

        /**
         * ID of auction.
         */
        static int auction_id;

    private:
        /**
         * Update the vector of robots.
         *
         * @param int id: The ID of the robot to update.
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
         * Update the information of a frontier auction in the private vector.
         *
         * @param int id: ID of the auction.
         * @param int robot: ID of robot giving a bid.
         * @param double bid: Bid of the robot.
         * @param Pose frontier: Frontier that is at auction.
         */
        void UpdateFrAuction(int id, int robot, double bid, Pose frontier);

        /**
         * Update the information of a docking station auction in the private vector.
         *
         * @param int id: ID of the auction.
         * @param int robot: ID of robot giving a bid.
         * @param int ds: ID of the docking station.
         * @param ds_state_t state: State of the docking station.
         * @param double bid: Bid of the robot.
         * @param Pose pose: Position of docking station.
         */
        void UpdateDsAuction(int id, int robot, int ds, ds_state_t state, double bid, Pose pose);

        /**
         * Check all auctions.
         * If this robot is winner of an auction that timed out, take action.
         */
        void CheckAuctions();

        /**
         * Store a new frontier auction in the private vector.
         */
        void StoreNewFrAuction(int id, double bid, int winner, usec_t time, bool open, Pose pose);

        /**
         * Store a new docking station auction in the private vector.
         */
        void StoreNewDsAuction(int id, double bid, int winner, usec_t time, bool open, int ds_id);

        /**
         * Send a broadcast message for a frontier auction.
         *
         * @param int id: ID of the auction.
         */
        void BroadcastFrAuction(int id);

        /**
         * Send a broadcast message for a docking station auction.
         *
         * @param int id: ID of the auction.
         */
        void BroadcastDsAuction(int id);

        /**
         * Send a beacon message about the robot.
         */
        void BroadcastBeacon();

        /**
         * Get the docking station object from the private vector.
         *
         * @param int id: The ID of the docking station
         */
        ds_t GetDs(int id);

        /**
         * Get the state of a docking station.
         *
         * @param int id: The ID of the docking station.
         * @return ds_state_t: The state of the docking station.
         */
        ds_state_t DsState(int id);

        /**
         * Get the position of a docking station.
         *
         * @param int id: The ID of the docking station.
         * @return Pose: The position of the docking station.
         */
        Pose DsPose(int id);

        /**
         * Set a docking station to the state occupied.
         *
         * @param int id: The ID of the docking station.
         */
        void DsOccupied(int id);

        /**
         * Get the docking station that is closest to the robots current location (preferably vacant).
         *
         * @param Pose pose: The robots current location.
         * @return int: The ID of the docking station.
         */
        int ClosestDsId(Pose pose);

        /**
         * Calculate the bid for a docking station.
         *
         * @param int ds: The ID of the docking station.
         * @param Pose pose: The position of the robot.
         * @return double: The bid.
         */
        double DockingBid(int ds, Pose pose);

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
         * Vector containing all docking stations.
         */
        vector<ds_t> dss;

        /**
         * Vector containing all frontier auctions this robot participated in.
         */
        vector<fr_auction_t> fr_auctions;

        /**
         * Vector containing all docking station auctions this robot participated in.
         */
        vector<ds_auction_t> ds_auctions;

        /**
         * Timeout for sending beacons. A beacon will only be send, after this timeout expired.
         */
        usec_t to_beacon;
    };
}

#endif
