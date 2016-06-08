#include "eae.hh"
#include "eae_coordination.hh"
#include "eae_robot.hh"
#include "region.hh"

using namespace Stg;
using namespace std;
using namespace eae;

/**
 * Initialize auction id.
 */
int Coordination::auction_id = 0;

/**
 * Initialization function that is called by Stage when the model starts up.
 */
extern "C" int Init(Model* mod, CtrlArgs* args)
{
    new Robot((ModelPosition*)mod);

    return 0; // ok
}
