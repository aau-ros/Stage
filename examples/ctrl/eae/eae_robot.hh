#ifndef EAE_ROBOT_H
#define EAE_ROBOT_H

#include "eae.hh"
#include "eae_coordination.hh"
#include "eae_gridmap.hh"
#include "eae_logoutput.hh"

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
     * @todo: Find optimal weight W5.
     */
    const int W1 = 10;
    const int W2 = 0;
    const int W3 = 9;
    const int W4 = 20;
    const int W5 = 10;

    /**
     * SOC at which the battery is considered as full.
     */
    const double CHARGE_FULL = 0.99;

    /**
     * SOC at which the robot will head home.
     */
    const double CHARGE_TURN = 0.45;

    /**
     * The power provided by the charger in watts.
     *
     * @todo: Read from world file instead.
     */
    const int WATTS_CHARGE = 1000;

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
     * Distance that the robot has to travel until another map update is performed.
     */
    const double MAP_UPDATE_DIST = 1.41;

    /**
     * A class that defines the behavior of a robot.
     */
    class Robot
    {
    public:
        /**
         * Constructor.
         *
         * @param ModelPosition* pos: The instantiated position model of the robot.
         * @param int robots: The number of robots in this simulation.
         * @param int dss: The number of docking stations in this simulation.
         */
        Robot(ModelPosition* pos, int robots, int dss);

        /**
         * Initialize the robot.
         */
        void Init();

        /**
         * Exploration routine, drives the robots towards unknown space.
         */
        void Explore();

        /**
         * Move the robot to the pose stored in the private variable goal.
         */
        void Move();

        /**
         * Move the robot to a given goal.
         *
         * @param Pose to: The goal where the robot should move to.
         * @param double bid: The bid the robot submitted for that goal.
         */
        void Move(Pose to, double bid);

        /**
         * Calculate the bid for a frontier.
         *
         * @param Pose frontier: The frontier to calculate the bid for.
         */
        double CalcBid(Pose frontier);

        /**
         * Return the camera object.
         *
         * @return OrthoCamera*: A pointer to the camera object of the robot.
         */
        OrthoCamera* GetCam();

        /**
         * Return the robot ID.
         *
         * @return int: The robot ID.
         */
        int GetId();

        /**
         * Return the state of the robot.
         *
         * @return robot_state_t: The state of the robot.
         */
        robot_state_t GetState();

        /**
         * Set the state of the robot.
         *
         * @param robot_state_t state: The state of the robot.
         */
        void SetState(robot_state_t state);

        /**
         * Set the position of the robot.
         *
         * @param Pose pose: The position of the robot.
         */
        void SetPose(Pose pose);

        /**
         * Get the position of the robot.
         *
         * @return Pose: The position of the robot.
         */
        Pose GetPose();

        /**
         * Check whether there is still a goal in the queue.
         *
         * @return bool: True if there is still a valid goal in the queue, false otherwise.
         */
        bool GoalQueue();

        /**
         * Start a docking procedure.
         *
         * @param ds_t ds: The docking station to dock at.
         * @param double bid: The bid that was made for that docking station.
         */
        void Dock(ds_t ds, double bid);

        /**
         * Estimate the remaining time that the robot can drive with its current battery.
         *
         * @return double: The remaining time in seconds.
         */
        double RemainingTime();

        /**
         * Estimate the remaining time the robot would need from now on to fully charge its battery.
         *
         * @return double: The remaining charge time in seconds.
         */
        double RemainingChargeTime();

        /**
         * Check whether the robot's battery is fully charged or not.
         *
         * @return bool: True if the robot's battery charge is above or equal to CHARGE_FULL.
         */
        bool FullyCharged();

        /**
         * Check whether the robot is currently on its way for recharging.
         *
         * @param ds_t& at: The docking station where the robot is docking at. It is set only if the robot is currently on its way for recharging.
         * @return bool: True if the robot is on its way for recharging, false otherwise.
         */
        bool Docking(ds_t& at);

        /**
         * Get the grid map.
         *
         * @return GridMap*: The grid map.
         */
        GridMap* GetMap();

        /**
         * Get a list of all frontiers the robot can reach with its current battery level.
         *
         * @return vector< vector <int> >: A vector containing an element for every frontier. Each element ist a vector with the two coordinates of that frontier.
         */
        vector< vector <int> > FrontiersReachable();

        /**
         * Update the local grid map with a given map.
         * Only non existent or unknown cells in the local map will be written.
         *
         * @param GridMap* map: The grid map.
         */
        void UpdateMap(GridMap* map);

    private:
        /**
         * Update the map with local sensor readings.
         */
        void UpdateMap();

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
         * Estimate the remaining distance that the robot can drive with its current battery.
         *
         * @return double: The remaining distance in meters.
         */
        double RemainingDist();

        /**
         * Write a line with the current status to the log file.
         */
        void Log();

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
         * Callback function that is called when the fiducial sensor detects something.
         *
         * @param ModelFiducial* fid: The instantiated fiducial model of the robot.
         * @param Robot* robot: The instantiated robot object which attached the callback.
         *
         * @return int: Returns 0.
         */
        static int FiducialUpdate(ModelFiducial* fid, Robot* robot);

        /**
         * Identifier of the robot.
         */
        int id;

        /**
         * GridMap object for internal representation of the map.
         */
        GridMap* map;

        /**
         * LogOutput object for logging of data to log files.
         */
        LogOutput* log;

        /**
         * The fiducial sensor of the robot for detecting docking stations.
         */
        ModelFiducial* fid;

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
         * The next goal that the robot navigates to, once it reaches the current goal.
         * This gets set when the robot wins an auction but is currently still navigating.
         */
        Pose goal_next;

        /**
         * The bid the robot submitted for the next goal.
         * This makes sure the robot stores the next goal where it submitted its highest bid.
         */
        double goal_next_bid;

        /**
         * The docking station that the robot selected for recharging.
         */
        ds_t ds;

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

        /**
         * Store position when last map update was made.
         * This helps to reduce the map update frequency to improve performance.
         */
        Pose last_pose;
    };
}

#endif
