#ifndef EAE_ROBOT_H
#define EAE_ROBOT_H

#include "eae.hh"
#include "eae_coordination.hh"
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
     * See also libstage/model_position.cc line 88.
     */
    static const double WATTS_KGMS = 2.0;

    /**
     * Power consumption in watts when robot is stationary.
     * See also libstage/model_position.cc line 89.
     */
    static const double WATTS = 9.2;

    /**
     * Distance that the robot has to travel until another map update is performed.
     */
    const double MAP_UPDATE_DIST = 1.41;

    /**
     * The amount the robot has to turn until another map update is performed.
     */
    const double MAP_UPDATE_ANGLE = PI/4;

    /**
     * Rate for charging robot.
     * This is supposed to be one second since the unit of power is watts (= joules / second).
     */
    const usec_t CHARGE_RATE = 1000000;

    /**
     * The amount the robot turns away from an obstacle in radians.
     */
    const double avoid_turn = PI/8;

    /**
     * The number of iterations a robot keeps avoiding an obstacle after passing it.
     * This makes sure that the robot does not crash into an obstacle when it is still next to it.
     */
    const int avoid_duration = 10;

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
         * Enqueue at a docking station.
         *
         * @param ds_t ds: The docking station to queue at.
         * @param double bid: The bid that was made for that docking station.
         */
        void DockQueue(ds_t ds, double bid);

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
         * @param ds_t& at: The docking station where the robot is charging at. It is set only if the robot is currently charging.
         *
         * @return bool: True if the robot is charging, false otherwise.
         */
        bool Charging(ds_t& at);

        /**
         * Check whether the robot is currently on its way for recharging.
         *
         * @param ds_t& at: The docking station where the robot is docking at. It is set only if the robot is currently on its way for recharging.
         *
         * @return bool: True if the robot is on its way for recharging, false otherwise.
         */
        bool Docking(ds_t& at);

        /**
         * Check whether the robot is currently waiting for recharging.
         *
         * @param ds_t& at: The docking station the robot is waiting for. It is set only if the robot is currently waiting.
         *
         * @return bool: True if the robot is waiting, false otherwise.
         */
        bool Queueing(ds_t& at);

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
         * Check whether the robot is currently following a planned path.
         *
         * @return bool: True if the robot has a valid path, false otherwise.
         */
        bool HasPlan();

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

        /**
         * The power at which the battery gets charged at a docking station.
         */
        watts_t charging_watt;

        /**
         * The latest time at which the robot was recharged.
         */
        usec_t last_charge;

        /**
         * Countdown for recovering from a crash.
         */
        int stall_count;

        /**
         * Countdown for avoiding an obstacle.
         * It stores the number of iterations a robot sticks to its decision of avoiding an obstacle in a certain direction.
         */
        int avoid_count;

        /**
         * Direction in which the robot is avoiding an obstacle.
         */
        double avoid_direction;

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
    };
}

#endif
