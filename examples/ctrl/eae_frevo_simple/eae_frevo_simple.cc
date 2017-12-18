#include "eae_frevo_simple.hh"
#include "eae_frevo_simple_robot.hh"

using namespace Stg;
using namespace std;
using namespace eae_frevo_simple;

/**
 * Initialization function that is called by Stage when the model starts up.
 */
extern "C" int Init(Model* mod, CtrlArgs* args)
{
    new Robot((ModelPosition*)mod);

    return 0; // ok
}
