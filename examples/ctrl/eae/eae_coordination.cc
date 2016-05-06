#include "eae.hh"

using namespace Stg;
using namespace std;

namespace eae
{
    Coordination::Coordination()
    {
        // initialize wifi adapter
        wifi = (ModelWifi*)pos->GetChild("wifi:0");
        wifi->AddCallback(Model::CB_UPDATE, (model_callback_t)WifiUpdate, this);
        wifi->comm.SetReceiveMsgFn(ProcessMessage);
        wifi->Subscribe();
    }

    bool Coordination::FrontierAuction(Pose frontier, double bid)
    {
        WifiMessageAuction msg;
        msg.type = MSG_FRONTIER_AUCTION;
        msg.auction = 0;
        msg.robot = 0;
        msg.frontier = frontier;

        WifiMessageBase* base_ptr = &msg;
        wifi->comm.SendBroadcastMessage(base_ptr);
    }

    int Coordination::WifiUpdate(ModelWifi* wifi, Robot* robot)
    {
        // visualize wifi connections
        wifi->DataVisualize(robot->cam);

        return 0; // run again
    }

    void Coordination::ProcessMessage(WifiMessageBase* incoming)
    {
        WifiMessage* msg = dynamic_cast<WifiMessage*>(incoming);

        switch(incoming->type) /////////////////////////////////////////////// continue here
        if(msg)
        {
            //printf("Robot [%u]: Neighbor [%u] is at (%.2f %.2f) and heading (%.2f)\n",
            //my_mesg->GetRecipientId(), my_mesg->GetSenderId(), my_mesg->gpose.x, my_mesg->gpose.y, my_mesg->gpose.a );
        }
        delete incoming;
    }
}
