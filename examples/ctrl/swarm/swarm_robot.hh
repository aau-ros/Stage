#ifndef SWARM_ROBOT_H
#define SWARM_ROBOT_H

#include "swarm.hh"
#include "swarm_coordination.hh"
#include "swarm_ds.hh"
#include "swarm_graph.hh"
#include "swarm_gridmap.hh"
#include "swarm_logoutput.hh"
#include "swarm_wifimessage.hh"

using namespace Stg;
using namespace std;

/**
 * Namespace for energy aware exploration (swarm).
 */
namespace swarm
{
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
     * Number of sectors for which to compute inputs for evolutionary controller.
     */
    const int SECTORS = 4;

    /**
     * TODO: Obstacle avoidance.
     */
    /*static const double cruisespeed = 0.4;
    static const double avoidspeed = 0.05;
    static const double avoidturn = 0.5;
    static const double minfrontdistance = 1.2; // 0.6
    static const double stopdist = 0.3;
    static const int avoidduration = 10;
    static const double frontsector = PI / 2;*/

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
         * Determine goal for the robot and store in private member variable.
         */
        void Explore();

        /**
         * Move the robot to the pose stored in the private variable goal.
         */
        void Move();

        /**
         * Set a goal for the robot.
         *
         * @param Pose to: The goal where the robot should move to.
         */
        void SetGoal(Pose to);

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
         * Get the grid map.
         *
         * @return GridMap*: The grid map.
         */
        GridMap* GetMap();

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
         * Get the name of the bitmap of the underlying map.
         *
         * @return string: The name of the map.
         */
        string MapName();
        
        /**
         * Compute the obstacle density in a given sector using the laser scan.
         * Assumption: 360° FOV.
         * 
         * @param int sector: The sector for which to compute the density, 0 starts from behind in mathematically positive direction.
         * @return double: The number of occupied samples (i.e. range < laser range) relative to the total number of samples in a sector.
         */
        double ObstacleDensity(int sector);
        
        /**
         * Compute the robot density in a given sector using positions retrieved over wifi.
         * 
         * @param int sector: The sector for which to compute the density, 0 starts from behind in mathematically positive direction.
         * @return double: The number of robots in each direction relative to all known robots.
         * @todo: Take into account the distance of the robots.
         */
        double RobotDensity(int sector);

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
         * Number of robots in the simulation.
         */
        int num_robots;
        
        /**
         * TODO: Obstacle avoidance.
         */
         /*int avoidcount;
         int randcount;*/
    };
}

#endif
