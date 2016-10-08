#ifndef EAE_ROBOT_H
#define EAE_ROBOT_H

#include "eae.hh"
#include "eae_coordination.hh"
#include "eae_ds.hh"
#include "eae_graph.hh"
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
    const double CHARGE_FULL = 0.95;

    /**
     * SOC to which the battery will be charged.
     */
    const double CHARGE_UNTIL = 0.99;

    /**
     * SOC at which the robot will head home.
     */
    const double CHARGE_TURN = 0.5;

    /**
     * Power consumption of the robot when it is stationary.
     */
    static const double POWER_S = 11.9;

    /**
     * Power consumption of the robot when it is moving.
     */
    static const double POWER_M = 22.15;

    /**
     * Distance that the robot has to travel until another map update is performed.
     */
    const double MAP_UPDATE_DIST = 1.41;

    /**
     * The amount the robot has to turn until another map update is performed.
     */
    const double MAP_UPDATE_ANGLE = PI/4;

    /**
     * Allowed distance between two points for them to be still at the same location.
     */
    const double EPSILON = 0.1;

    /**
     * How many times the robot should try turning to help computing a path before giving up.
     */
    const int TURN_TRIALS = 4;

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
         */
        Robot(ModelPosition* pos);

        /**
         * Destructor.
         */
        ~Robot();

        /**
         * Initialize the robot.
         */
        void Init();

        /**
         * Exploration routine, drives the robots towards unknown space.
         */
        void Explore();

        /**
         * Move the robot to the pose stored in the private variable goal
         * following the planned path stored in the private variable path.
         *
         * @param bool clear: If true, remove old path and plan new one to goal. Default false.
         */
        void Move(bool clear=false);

        /**
         * Set a goal for the robot.
         *
         * @param Pose to: The goal where the robot should move to.
         * @param double bid: The bid the robot submitted for that goal.
         */
        void SetGoal(Pose to, double bid);

        /**
         * Set the current goal for the robot to the next goal.
         */
        void SetGoalNext();

        /**
         * Calculate the bid for a frontier.
         *
         * @param Pose frontier: The frontier to calculate the bid for.
         */
        double CalcBid(Pose frontier);

        /**
         * Compute a path from start to goal using the A* algorithm
         * and store the result as a graph in the private variable path.
         *
         * @param Pose start_pose: The starting point.
         * @param Pose goal_pose: The end point.
         *
         * @return bool: Success of path generation.
         */
        bool Plan(Pose start_pose, Pose goal_pose);

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
         * Get the position of the robot.
         *
         * @return Pose: The position of the robot.
         */
        Pose GetPose();

        /**
         * Get the DS which was last selected by the robot.
         *
         * @return Ds*: Pointer to the selected DS.
         */
        Ds* GetDs();

        /**
         * Check whether there is still a goal in the queue.
         *
         * @return bool: True if there is still a valid goal in the queue, false otherwise.
         */
        bool GoalQueue();

        /**
         * Start a docking procedure.
         *
         * @param Ds* ds: Pointer to the docking station to dock at.
         * @param double bid: The bid that was made for that docking station.
         */
        void Dock(Ds* ds, double bid);

        /**
         * Enqueue at a docking station.
         *
         * @param Ds* ds: Pointer to the docking station to queue at.
         * @param double bid: The bid that was made for that docking station.
         */
        void DockQueue(Ds* ds, double bid);

        /**
         * Stop recharging and continue exploration.
         */
        void UnDock();

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
         * Estimate the distance that the robot can drive with full battery.
         *
         * @return double: The distance in meters.
         */
        double MaxDist();

        /**
         * Check whether the robot's battery is fully charged or not.
         *
         * @return bool: True if the robot's battery charge is above or equal to CHARGE_FULL.
         */
        bool FullyCharged();

        /**
         * Check whether the robot is currently charging.
         *
         * @return Ds*: The docking station where the robots is currently charging. NULL if it is not charging.
         */
        Ds* Charging();

        /**
         * Check whether the robot is currently on its way for recharging.
         *
         * @return Ds*: The docking station where the robots is currently docking. NULL if it is not docking.
         */
        Ds* Docking();

        /**
         * Check whether the robot is currently waiting for recharging.
         *
         * @return Ds*: The docking station where the robots is currently waiting. NULL if it is not waiting.
         */
        Ds* Queueing();

        /**
         * Check wheter the robot is currently waiting for recharging at a docking station.
         * 
         * @return Ds*: The docking station where the robots is currently waiting. NULL if it is not waiting.
         */
        Ds* QueueingAtDs();

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
         * Get a list of all frontiers the robot can reach from a given position.
         *
         * @param Pose pos: The position from where to start looking.
         * @param double range: The range that the robot can travel.
         * @param bool ds: Whether the given position is a docking station or not.
         *
         * @return vector< vector <int> >: A vector containing an element for every frontier. Each element ist a vector with the two coordinates of that frontier.
         */
        vector< vector <int> > FrontiersReachable(Pose pos, double range, bool ds);

        /**
         * Compute the distance from the current location to a point.
         *
         * @param double to_x: X-coordinate of end point.
         * @param double to_y: Y-coordinate of end point.
         *
         * @return int: The distance, -1 if plan fails.
         */
        int Distance(double to_x, double to_y);

        /**
         * Compare two points whether the X and Y-coordinates are within EPSILON.
         *
         * @param Pose point1: The first point.
         * @param Pose point2: The second point.
         *
         * @return bool: True if the two points are within distance EPSILON of each other.
         */
        bool SamePoint(Pose point1, Pose point2);

        /**
         * Continue exploration after finalizing too early.
         */
        void Continue();

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
         * Estimate the remaining distance that the robot can drive with a given battery charge.
         *
         * @param joules_t charge: The charge of the robots battery.
         *
         * @return double: The remaining distance in meters.
         */
        double RemainingDist(joules_t charge);

        /**
         * Write a line with the current status to the log file.
         */
        void Log();

        /**
         * Finalize the exploration.
         */
        void Finalize();

        /**
         * Set the motor velocities and avoid obstacles.
         *
         * @param double direction: The relative direction the robot would go without obstacles.
         */
        void SetMotorSpeed(double direction);

        /**
         * Get the name of the bitmap of the underlying map.
         *
         * @return string: The name of the map.
         */
        string MapName();

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
         * Callback function that is called when the docking station supplies energy to the robot.
         *
         * @param Model* mod: The instantiated docking station model.
         * @param Robot* robot: The instantiated robot object which attached the callback.
         *
         * @return int: Returns 0.
         */
        static int ChargingUpdate(Model* mod, Robot* robot);

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
         * The laser range sensor (lidar) of the robot for mapping obstacles.
         */
        ModelRanger* laser;

        /**
         * The sonar range sensor of the robot for avoiding obstacles.
         */
        ModelRanger* sonar;

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
         * The previous goal that the robot navigated to.
         */
        Pose goal_prev;

        /**
         * The bid the robot submitted for the next goal.
         * This makes sure the robot stores the next goal where it submitted its highest bid.
         */
        double goal_next_bid;

        /**
         * An intermediate goal that is part of the path calculated by the path planner.
         */
        Pose goal_step;

        /**
         * The docking station that the robot selected for recharging.
         */
        Ds* ds;

        /**
         * The previous docking station selected by the robot.
         */
        Ds* ds_prev;

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

        /**
         * The power at which the battery gets charged at a docking station.
         */
        watts_t charging_watt;

        /**
         * The latest time at which the robot was recharged.
         */
        usec_t last_charge;

        /**
         * The path planned by the path planner.
         */
        Graph* path;

        /**
         * Whether the planned path is valid or not.
         */
        bool valid_path;

        /**
         * The accumulated time that the robot has to wait for recharging.
         */
        int waiting_time;

        /**
         * The time in seconds when the robot started to wait for recharging.
         */
        int waiting_start;

        /**
         * If the robot cannot compute a path, it turns 90° and tries again.
         * This variable stores how many times the robot tried turning.
         * It is reset to 0 if it was successfull.
         */
        int turning;

        /**
         * This variable stores the angle the robot tried turning already.
         * It is reset to 0 if it was successfull.
         */
        double turned;

        /**
         * Number of robots in the simulation.
         */
        int num_robots;
    };
}

#endif
