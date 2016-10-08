#include "eae_ds.hh"

using namespace Stg;
using namespace std;

namespace eae
{
    Ds::Ds()
    {
        // initialize member variables
        id = 0;
        state = STATE_VACANT;
        pose = Pose();
        model = NULL;
        robots = 0;
    }
}
