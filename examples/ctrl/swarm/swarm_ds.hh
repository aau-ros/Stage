#ifndef SWARM_DS_H
#define SWARM_DS_H

#include "swarm.hh"

using namespace Stg;
using namespace std;

/**
 * Namespace for energy aware exploration (swarm).
 */
namespace swarm
{
    /**
     * A class that defines the docking stations.
     */
    class Ds
    {
    public:
        /**
         * Constructor.
         */
        Ds();

        /**
         * ID of the docking station.
         */
        int id;

        /**
         * Location of the docking station.
         */
        Pose pose;

        /**
         * Stage model of the docking station.
         */
        Model* model;
    };
}

#endif
