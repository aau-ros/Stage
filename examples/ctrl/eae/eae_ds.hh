#ifndef EAE_DS_H
#define EAE_DS_H

#include "eae.hh"

using namespace Stg;
using namespace std;

/**
 * Namespace for energy aware exploration (eae).
 */
namespace eae
{
    /**
     * The current state of a docking station.
     */
    typedef enum{
        STATE_UNDEFINED_DS = 0,
        STATE_VACANT,
        STATE_OCCUPIED
    } ds_state_t;

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
         * State of the docking station.
         */
        ds_state_t state;

        /**
         * Location of the docking station.
         */
        Pose pose;

        /**
         * Stage model of the docking station.
         */
        Model* model;

        /**
         * Number of robots currently associated with the docking station.
         */
        int robots;
    };
}

#endif
