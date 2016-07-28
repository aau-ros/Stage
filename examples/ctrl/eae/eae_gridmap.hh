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
         * @param Pose pose: The initial position of the robot.
         * @param World* world: The world used for visualization.
         * @param int robot: The ID of the robot.
         */
        GridMap(Pose pose, World* world, int robot);

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
         * @return GridMap*: A map containing only the updated cells.
         */
        GridMap* Clear(Pose pos, vector<meters_t> scan);

        /**
         * Get a list of all frontiers.
         *
         * @return vector< vector <int> >: A vector containing an element for every frontier. Each element ist a vector with the two coordinates of that frontier.
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
         */
        int ExploredCells();

        /**
         * Get the amount of explored area combined from all robots.
         *
         * @return double: The explored area in square meters.
         */
        double Explored();

        /**
         * Assignment operator.
         *
         * @param GridMap& toCopy: The object that should be copied.
         * @return GridMap&: A pointer to this object.
         */
        GridMap& operator=(const GridMap& toCopy);

        /**
         * Convert meters to cell indexes.
         *
         * @param Pose m: A position in meters.
         * @return grid_cell_t: The cell index in x and y coordinates.
         */
        grid_cell_t M2C(Pose m);

        /**
         * Convert cell indexes to meters.
         *
         * @param grid_cell_t c: A cell index in x and y coordinates.
         * @return Pose: The position in meters.
         */
        Pose C2M(grid_cell_t c);

        /**
         * Convert a distance in cells to a distance in meters.
         *
         * @param unsigned int c: A distance in number of cells.
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
         * Converts the local grid map into a uint8_t*
         * containing only 1s and 9s where
         * 1 represents a free cell and
         * 9 represents an occupied cell.
         *
         * @return uint8_t*: The converted grid map.
         */
        uint8_t* Rasterize();

    private:
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
    };
}

#endif
