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
     */
    const int W1 = 10;
    const int W2 = 0;
    const int W3 = 9;
    const int W4 = 20;

    /**
     * SOC at which the battery is considered as full.
     */
    const double CHARGE_FULL = 0.99;

    /**
     * SOC at which the robot will head home.
     */
    const double CHARGE_TURN = 0.45;

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
         * Exploration routine, drives the robots towards unknown space.
         */
        void Explore();

        /**
         * Move the robot to the pose stored in the private variable goal.
         */
        void Move();

        /**
         * Move the robot to a given position.
         *
         * @param Pose to: The position where the robot should move to.
         */
        void Move(Pose to);

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
         * Get the grid map.
         *
         * @return GridMap*: The grid map.
         */
        GridMap* GetMap();

        /**
         * Update the grid map. Only non existent or unknown cells will be written.
         *
         * @param GridMap* map: The grid map.
         */
        void UpdateMap(GridMap* map);

    private:
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
         * @return double: The remaining distance.
         */
        double RemainingDist();

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
    };
}

#endif
