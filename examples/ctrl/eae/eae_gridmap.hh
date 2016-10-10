#ifndef EAE_GRIDMAP_H
#define EAE_GRIDMAP_H

#include "eae.hh"

using namespace Stg;
using namespace std;

/**
 * Namespace for energy aware exploration (eae).
 */
namespace eae
{
    /**
     * Plan paths through unknown areas.
     */
    const bool PLAN_UNKNOWN = false;

    /**
     * A class for the internal representation of the map.
     */
    class GridMap
    {
    public:
        /**
         * Constructor.
         *
         * @param ModelPosition* pos: The position model of the robot.
         * @param int robot: The ID of the robot.
         */
        GridMap(ModelPosition* pos, int robot);

        /**
         * Destructor.
         */
        ~GridMap();

        /**
         * Visualize the map in the command line.
         *
         * @param Pose pose: The current position of the robot.
         */
        void Visualize(Pose pose);

        /**
         * Visualize the map in the stage GUI.
         *
         * @param Pose pose: The current position of the robot.
         */
        void VisualizeGui(Pose pose);

        /**
         * Insert a value into the map.
         * If the specified coordinates exceed the map dimension, the map will be extended.
         *
         * @param int x: X-coordinate of the cell.
         * @param int y: Y-coordinate of the cell.
         * @param grid_cell_v val: Value to insert.
         */
        void Insert(int x, int y, grid_cell_v val);

        /**
         * Mark all cells visible from the robot's current position as occupied or free, depending on laser scan data.
         *
         * @param Pose pos: The position of the robot.
         * @param vector<meters_t> scan: The laser scan data.
         *
         * @return GridMap*: A map containing only the updated cells.
         */
        GridMap* Clear(Pose pos, vector<meters_t> scan);

        /**
         * Get a list of frontiers in range of the robot.
         * Not all frontiers might actually be reachable since
         * the robot also needs to return to the docking station.
         *
         * @param int range: The range that the robot can travel.
         * @param int max_frontiers: Number of frontiers to look for, default -1 for unlimited.
         *
         * @return vector< vector <int> >: A vector containing one element. It is a vector with the two coordinates of that frontier.
         */
        vector< vector <int> > ReachableFrontiers(int range, int max_frontiers=-1);

        /**
         * Get a list of all frontiers.
         *
         * @return vector< vector <int> >: A vector containing an element for every frontier. Each element is a vector with the two coordinates of that frontier.
         */
        vector< vector <int> > Frontiers();

        /**
         * Update the grid map. Only non existent or unknown cells will be written.
         *
         * @param GridMap* map: A GridMap object to take the data from.
         */
        void Update(GridMap* map);

        /**
         * Get the amount of explored area combined from all robots.
         *
         * @return int: The number of explored grid cells.
         *
         * @todo: store already known area and iterate only over newly explored area
         */
        int ExploredCells();

        /**
         * Get the amount of explored area combined from all robots.
         *
         * @return double: The explored area in square meters.
         */
        double Explored();

        /**
         * Compute distance between two points.
         *
         * @param double from_x: X-coordinate of starting point.
         * @param double from_y: Y-coordinate of starting point.
         * @param double to_x: X-coordinate of end point.
         * @param double to_y: Y-coordinate of end point.
         *
         * @return int: The distance, -1 if plan fails.
         */
        int Distance(double from_x, double from_y, double to_x, double to_y);

        /**
         * Generate a path from start to goal using the A* algorithm.
         *
         * @param Pose start_pose: The starting point.
         * @param Pose goal_pose: The end point.
         * @param vector<ast::point_t>* path: The resulting path will be stored here.
         *
         * @return bool: Success of path generation.
         */
        bool AStar(Pose start_pose, Pose goal_pose, vector<ast::point_t>* path);

        /**
         * Convert meters to cell indexes.
         *
         * @param Pose m: A position in meters.
         *
         * @return grid_cell_t: The cell index in x and y coordinates.
         */
        grid_cell_t M2C(Pose m);

        /**
         * Convert cell indexes to meters.
         *
         * @param grid_cell_t c: A cell index in x and y coordinates.
         *
         * @return Pose: The position in meters.
         */
        Pose C2M(grid_cell_t c);

        /**
         * Convert a distance in cells to a distance in meters.
         *
         * @param unsigned int c: A distance in number of cells.
         *
         * @return double: The distance in meters.
         */
        double C2M(unsigned int c);

        /**
         * Get the width of the map.
         *
         * @return unsigned int: The width of the map in grid cells.
         */
        unsigned int Width();

        /**
         * Get the height of the map.
         *
         * @return unsigned int: The height of the map in grid cells.
         */
        unsigned int Height();

        /**
         * Get the size of the map in number of bytes.
         *
         * @return unsigned int: The size of the map.
         */
        unsigned int Size();

        /**
         * Converts the local grid map into a uint8_t* where
         * 1-5 represent free cells (with different cost) and
         * 9 represents an occupied cell.
         *
         * @return uint8_t*: The converted grid map.
         */
        uint8_t* Rasterize();

        /**
         * Read the value of one grid cell.
         * The given coordinates will be converted to the correct indices.
         * Out of range exceptions are not handled!
         *
         * @param int x: X-coordinate of the cell.
         * @param int y: Y-coordinate of the cell.
         *
         * @return grid_cell_v: The value of the grid cell.
         */
        grid_cell_v Read(int x, int y);

    private:
        /**
         * Write a value to a grid cell.
         * The given coordinates will be converted to the correct indices.
         * Out of range exceptions are not handled!
         *
         * @param int x: X-coordinate of the cell.
         * @param int y: Y-coordinate of the cell.
         * @param grid_cell_v val: The value of the grid cell.
         */
        void Write(int x, int y, grid_cell_v val);

        /**
         * Determines if a cell is a frontier cell.
         *
         * @param int x: X-coordinate of the cell.
         * @param int y: Y-coordinate of the cell.
         *
         * @return bool: True, if it is a frontier cell, false otherwise.
         */
        bool Frontier(int x, int y);

        /**
         * Variable holding the grid map.
         */
        vector< vector <grid_cell_v> > grid;

        /**
         * Variables for the map dimensions.
         */
        unsigned int x_dim;    // number of cells in x-dimension
        unsigned int y_dim;    // number of cells in y-dimension
        unsigned int x_offset; // offset of initial robot position in x-dimension
        unsigned int y_offset; // offset of initial robot position in y-dimension
        int x_min;    // minimum x-coordinate in map
        int x_max;    // maximum x-coordinate in map
        int y_min;    // minimum y-coordinate in map
        int y_max;    // maximum y-coordinate in map

        /**
         * Visualizer objects.
         */
        Model* vis_frontier;
        Model* vis_free;
        Model* vis_unknown;
        Model* vis_occupied;

        /**
         * Robot ID.
         */
        int robot;

        /**
         * Resolution of the map.
         */
        double resolution;

        /**
         * Position model of the robot.
         */
        ModelPosition* pos;

        /**
         * The rasterized grid map.
         */
        uint8_t* raster;

        /**
         * Whether the rasterized grid map is still valid.
         */
        bool raster_valid;
    };
}

#endif
