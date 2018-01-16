#ifndef FREVO_ROBOT_H
#define FREVO_ROBOT_H

#include "frevo.hh"
#include "frevo_gridmap.hh"
#include "frevo_logoutput.hh"

using namespace Stg;
using namespace std;

/**
 * Namespace for frevo.
 */
namespace frevo
{
    /**
     * Allowed distance between two points for them to be still at the same location.
     */
    const double EPSILON = 0.1;

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
         * Escape routine, drives the robots towards the closest emergency exit.
         */
        void Escape();

        /**
         * Move the robot to the given pose.
         *
         * @param Pose goal: The goal pose to move the robot to.
         */
        void Move(Pose* goal);

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

    private:
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
         * Sense whether the neighboring cells are free or occupied.
         * The data is taken from the floorplan model.
         * The result is stored in the private occupancy variable.
         */
        void Sense();
        
        /**
         * Sense whether the neighboring cells are free or occupied.
         * The data is taken from the lidar range values.
         * The result is stored in the private occupancy variable.
         */
        void LidarSense();

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
        
        static int LidarUpdate(ModelRanger* lidar, Robot* robot);

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
         * The position model of the robot.
         */
        ModelPosition* pos;

        /**
         * The camera object that is used for visualization.
         */
        OrthoCamera* cam;

        /**
         * The laser range sensor (lidar) of the robot for mapping obstacles.
         */
        ModelRanger* laser;

        /**
         * The goal where the robot navigates to.
         */
        Pose goal;

        /**
         * The previous goal that the robot navigated to.
         */
        Pose goal_prev;

        /**
         * An intermediate goal that is part of the path calculated by the path planner.
         */
        Pose goal_step;

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
         * Variable holding the occupancy values of the eight neighboring cells.
         * Index 0 corresponds to east, 1 north east, and so on.
         * A Value of 0 corresponds to a free cell, a value of 1 is occupied.
         * The array is filled in Sense().
         */
        uint8_t occupancy[8];
        
        /**
         * List of emergency exits (coordinates).
         */
        vector< vector<int> > exits;
    };
}

#endif
