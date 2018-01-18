#include "stage.hh"
using namespace Stg;

int LaserUpdate(ModelRanger* ranger);
    
// Stage calls this when the model starts up
extern "C" int Init( Model* mod, CtrlArgs* args )
{  
  ModelRanger* laser = (ModelRanger*)mod->GetChild("ranger:0");
  laser->AddCallback(Model::CB_UPDATE, (model_callback_t)LaserUpdate, NULL);
  laser->Subscribe();
  return 0; //ok
}

int LaserUpdate(ModelRanger* ranger){
  return 0; //ok
}

