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
     * Laser range in grid cells.
     */
    const int LASER_RANGE = 3;

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
         * @param grid_cell_t val: Value to insert.
         */
        void Insert(int x, int y, grid_cell_t val);

        /**
         * Mark all cells visible from the robot's current position as visited.
         *
         * @param Pose pos: The position of the robot.
         * @return GridMap*: A map containing only the updated cells.
         *
         * @todo: Don't always mark as free, but according to actual sensor reading.
         */
        GridMap* Clear(Pose pos);

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

    private:
        /**
         * Read the value of one grid cell.
         * The given coordinates will be converted to the correct indices.
         * Out of range exceptions are not handled!
         *
         * @param int x: X-coordinate of the cell.
         * @param int y: Y-coordinate of the cell.
         *
         * @return grid_cell_t: The value of the grid cell.
         */
        grid_cell_t Read(int x, int y);

        /**
         * Write a value to a grid cell.
         * The given coordinates will be converted to the correct indices.
         * Out of range exceptions are not handled!
         *
         * @param int x: X-coordinate of the cell.
         * @param int y: Y-coordinate of the cell.
         * @param grid_cell_t val: The value of the grid cell.
         */
        void Write(int x, int y, grid_cell_t val);

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
        vector< vector <grid_cell_t> > grid;

        /**
         * Variables for the map dimensions.
         */
        int x_dim;    // number of cells in x-dimension
        int y_dim;    // number of cells in y-dimension
        int x_offset; // offset of origin in x-dimension
        int y_offset; // offset of origin in y-dimension
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
