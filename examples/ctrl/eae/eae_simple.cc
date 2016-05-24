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
 * Start exploration when world is ready.
 */
static int Start(World* world, Robot* robot)
{
    robot->Explore();

    return -1; // run only once
}

/**
 * Initialization function that is called by Stage when the model starts up.
 */
extern "C" int Init(Model* mod, CtrlArgs* args)
{

    Robot* robot = new Robot((ModelPosition*)mod);

    World* world = mod->GetWorld();

    /*SuperRegion* region = new SuperRegion(world, point_int_t(0,0));
    region->AddBlock();
    region->DrawOccupancy();*/
    //world->AddBlockRect(3,3,1,1,1);
    //mod->Rasterize();
    //Gl::draw_centered_rect(3, 3, 1, 1);

    // don't start exploration before the world is ready
    world->AddUpdateCallback((world_callback_t)Start, robot);

    return 0; // ok
}
