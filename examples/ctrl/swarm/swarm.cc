#include "swarm.hh"
#include "swarm_robot.hh"

using namespace Stg;
using namespace std;
using namespace swarm;

/**
 * Initialization function that is called by Stage when the model starts up.
 */
extern "C" int Init(Model* mod, CtrlArgs* args)
{
    new Robot((ModelPosition*)mod);

    return 0; // ok
}
