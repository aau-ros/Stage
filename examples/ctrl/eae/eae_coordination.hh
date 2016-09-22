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
     * Policy of a robot for selecting a docking station.
     */
    typedef enum{
        POL_UNDEFINED = 0,
        POL_CLOSEST,
        POL_VACANT,
        POL_OPPORTUNE,
        POL_CURRENT,
        POL_CONNECTED,
        POL_COMBINED
    } pol_t;

    /**
     * Strings describing the policy.
     * Make sure they match the pol_t enum!
     */
    const string POL_STRING[] = {"undefined", "closest", "vacant", "opportune", "current", "connected", "combined"};

    /**
     * Possible coordination strategies for recharging at docking stations.
     *
     * @todo: Implement optimal coordination strategy.
     */
    typedef enum{
        CORD_MARKET = 0,
        CORD_GREEDY,
        CORD_OPT
    } cord_t;

    /**
     * Strings describing the coordination strategy.
     * Make sure they match the cord_t enum!
     */
    const string CORD_STRING[] = {"market", "greedy", "opt"};

    /**
     * Weights for the cost function.
     *
     * @todo: Find optimal weights.
     */
    const int L1 = 1;
    const int L2 = 3;
    const int L3 = 1;
    const int L4 = 5;

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
     * Tolerance when calculating the distance to the closest docking station.
     */
    const double DS_TOLERANCE = 2;

    /**
     * Auction type for frontiers.
     */
    typedef struct{
        int id;
        double highest_bid;
        int initiator;
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
        int initiator;
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
         * @param int ds: ID of the docking station to start the auction for.
         *
         * @return: Success of auction generation. True if new auction was started,
         * false if there was not enough time since last auction for the same docking station.
         */
        bool DockingAuction(Pose pose, int ds);

        /**
         * Send a map update to the other robots containing the complete map.
         */
        void BroadcastMap();

        /**
         * Send a map update to the other robots containing only part of the map.
         *
         * @param GridMap* local: The map to send.
         */
        void BroadcastMap(GridMap* local);

        /**
         * Returns the distance from a given pose to the closest goal another robot has been to.
         *
         * @param Pose pose: The pose to calculate the distance from.
         *
         * @return double: The distance to the closest goal.
         */
        double DistRobot(Pose pose);

        /**
         * Update or add a docking station to the private vector.
         *
         * @param int id: The ID of the docking station (as returned by the fiducial).
         * @param Pose pose: The position of the docking station.
         * @param ds_state_t state: The state of the docking station, default is undefined.
         * @param int change: The number of new or removed robots (the ones that selected this docking station), default 0.
         */
        void UpdateDs(int id, Pose pose, ds_state_t state=STATE_UNDEFINED_DS, int change=0);

        /**
         * Select a docking station for recharging.
         *
         * @param double range: The range of the robot, not required for all policies.
         * @param pol_t policy: Override the global policy for selecting a docking station.
         * @param int exclude: Exclude this docking station, default none.
         * @param bool end: Selection that takes place at end of exploration, only docking stations with frontiers in reach are of interest, default false.
         */
        ds_t SelectDs(double range=0, pol_t policy=POL_UNDEFINED, int exclude=0, bool end=false);

        /**
         * Set a docking station to the state vacant.
         *
         * @param int id: The ID of the docking station.
         */
        void DsVacant(int id);

        /**
         * Check if all other robots finished the exploration already.
         *
         * @return bool: True if all other robots are in state STATE_FINISHED.
         */
        bool Finished();

        /**
         * Check if a frontier has already been auctioned.
         *
         * @param Pose frontier: The frontier to check.
         *
         * @return bool: True if the frontier is in the list of auctioned frontiers.
         */
        bool OldFrontier(Pose frontier);

        /**
         * Get the wifi model.
         *
         * @return string: The name of the wifi model.
         */
        string GetWifiModel();

        /**
         * Get the coordination strategy.
         *
         * @return cord_t: The index of the coordination strategy.
         */
        cord_t GetStrategy();

        /**
         * Get the coordination strategy.
         *
         * @return string: A string describing the coordination strategy.
         */
        string GetStrategyString();

        /**
         * Get the policy for selecting docking stations.
         *
         * @return pol_t: The index of the policy.
         */
        pol_t GetPolicy();

        /**
         * Get the policy for selecting docking stations.
         *
         * @return string: A string describing the policy.
         */
        string GetPolicyString();

        /**
         * ID of auction.
         */
        static int auction_id;

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
         * Get the docking station closest to the robot's current position.
         *
         * @return ds_t: The closest docking station.
         */
        ds_t ClosestDs();

        /**
         * Get the docking station closest to the robot's current position that is vacant and in range of the robot.
         *
         * @param double range: The range of the robot.
         *
         * @return ds_t: The closest free docking station.
         */
        ds_t VacantDs(double range);

        /**
         * Get the docking station closest to the robots current location where there are frontiers nearby that the robot can reach after recharging. If this is not possible get the closest docking station where another docking station with frontiers nearby can be reached in the next step.
         *
         * @param double range: The range of the robot.
         * @param int exclude: Exclude this docking station.
         * @param bool end: Selection that takes place at end of exploration, only docking stations with frontiers in reach are of interest, default false.
         *
         * @return ds_t: The docking station.
         */
        ds_t OpportuneDs(double range, int exclude, bool end=false);

        /**
         * Get the docking station that the robot preveously selected if there are still frontiers/opportunities in range. Otherwise use opportune policy to select another one.
         *
         * @param double range: The range of the robot.
         *
         * @return ds_t: The docking station.
         */
        ds_t CurrentDs(double range);

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
         * @param double bid: Bid of the robot.
         */
        void UpdateDsAuction(int id, int robot, int ds, double bid);

        /**
         * Check all auctions.
         * If this robot is winner of an auction that timed out, take action.
         */
        void CheckAuctions();

        /**
         * Store a new frontier auction in the private vector.
         *
         * @param int id: ID of the auction.
         * @param double bid: Bid for the auction.
         * @param int initiator: ID of the robot who started the auction.
         * @param int winner: ID of the current highest bidder.
         * @param usec_t time: Timestamp of the auction.
         * @param bool open: Whether or not the auction is still running.
         * @param Pose pose: Position of the frontier.
         */
        void StoreNewFrAuction(int id, double bid, int initiator, int winner, usec_t time, bool open, Pose pose);

        /**
         * Store a new docking station auction in the private vector.
         *
         * @param int id: ID of the auction.
         * @param double bid: Bid for the auction.
         * @param int initiator: ID of the robot who started the auction.
         * @param int winner: The current highest bidder.
         * @param usec_t time: Timestamp of the auction.
         * @param bool open: Whether or not the auction is still running.
         * @param int ds_id: ID of the docking station.
         */
        void StoreNewDsAuction(int id, double bid, int initiator, int winner, usec_t time, bool open, int ds_id);

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
         *
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
         * Calculate the bid for a docking station.
         *
         * @param int ds: The ID of the docking station.
         *
         * @return double: The bid.
         */
        double DockingBid(int ds);

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
         * The coordination strategy for coordinating the recharging at docking stations.
         */
        cord_t strategy;

        /**
         * The policy for selecting a docking station for recharging.
         */
        pol_t policy;

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
