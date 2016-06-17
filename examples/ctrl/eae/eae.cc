#include "eae.hh"
#include "eae_coordination.hh"
#include "eae_robot.hh"

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
    // read number of robots and docking stations from world file (just for log output)
    Worldfile* wf = mod->GetWorld()->GetWorldFile();
    int robots = 0;
    robots = wf->ReadInt(0, "robots", robots);
    int dss = 0;
    dss = wf->ReadInt(0, "docking_stations", dss);

    new Robot((ModelPosition*)mod, robots, dss);

    return 0; // ok
}
