#include "eae.hh"

using namespace Stg;
using namespace std;
using namespace eae;


/**
 * Initialization function that is called by Stage when the model starts up.
 */
extern "C" int Init(Model* mod, CtrlArgs* args)
{
    Robot* robot = new Robot((ModelPosition*)mod);

    robot->Explore();

    return 0; // ok
}
