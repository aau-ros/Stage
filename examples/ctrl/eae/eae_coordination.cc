#include "eae_coordination.hh"

using namespace Stg;
using namespace std;

namespace eae
{
    Coordination::Coordination(ModelPosition* pos, Robot* robot)
    {
        // position model to access wifi
        this->pos = pos;

        // initialize wifi adapter
        wifi = (ModelWifi*)pos->GetChild("wifi:0");
        wifi->AddCallback(Model::CB_UPDATE, (model_callback_t)WifiUpdate, this);
        wifi->comm.SetReceiveMsgFn(ProcessMessage, this);
        wifi->Subscribe();

        // robot owning this object
        this->robot = robot;

        // beacon timeout
        to_beacon = 0;
    }

    void Coordination::FrontierAuction(Pose frontier, double bid)
    {
        // invalid bid
        if(bid == BID_INV){
            printf("invalid bid\n");
            return;
        }

        // increment auction id
        int id = ++auction_id;

        // create and store auction
        StoreNewAuction(id, bid, robot->GetId(), pos->GetWorld()->SimTimeNow(), true, frontier);

        // notify other robots
        BroadcastAuction(id);
    }

    void Coordination::BroadcastMap()
    {
        WifiMessageMap* msg = new WifiMessageMap(robot->GetMap());
        WifiMessageBase* base_ptr = msg;
        wifi->comm.SendBroadcastMessage(base_ptr);
    }

    double Coordination::DistRobot(Pose pose)
    {
        double dist = 0;
        double dist_temp;
        for(i_r=robots.begin(); i_r<robots.end(); ++i_r){
            dist_temp = pose.Distance(i_r->pose);
            if(dist_temp < dist || dist == 0)
                dist = dist_temp;
        }
        return dist;
    }

    void Coordination::UpdateRobots(int id, robot_state_t state, Pose pose)
    {
        //  i don't need to be on the list
        if(id == robot->GetId())
            return;

        // iterate through all robots
        for(i_r=robots.begin(); i_r<robots.end(); ++i_r){
            // found robot
            if(i_r->id == id){
                // update state and pose
                i_r->state = state;
                i_r->pose = pose;
                break;
            }
        }

        // robot not found
        if(i_r == robots.end()){
            // insert new robot into list
            robot_t robot;
            robot.id = id;
            robot.state = state;
            robot.pose = pose;
            robots.push_back(robot);
        }
    }

    void Coordination::UpdateMap(GridMap* map)
    {
        robot->UpdateMap(map);
    }

    void Coordination::UpdateAuction(int id, int robot, double bid, Pose frontier)
    {
        // invalid bid
        if(bid == BID_INV){
            printf("invalid bid\n");
            return;
        }

        // iterate through all auctions
        for(i_a=auctions.begin(); i_a<auctions.end(); ++i_a){
            // found auction
            if(i_a->id == id){
                // check if auction is still open and if bid is higher
                if(i_a->open && i_a->highest_bid < bid){
                    i_a->highest_bid = bid;
                    i_a->winner = robot;
                }
                break;
            }
        }

        // auction not found
        if(i_a == auctions.end()){
            // calculate bid
            double my_bid = this->robot->CalcBid(frontier);

            // invalid bid
            if(my_bid == BID_INV){
                printf("invalid bid\n");
                return;
            }

            // my bid is higher
            if(my_bid > bid){
                // store auction
                StoreNewAuction(id, my_bid, this->robot->GetId(), pos->GetWorld()->SimTimeNow(), true, frontier);

                // notify other robots
                BroadcastAuction(id);
            }

            // my bid is lower, just store auction
            else{
                StoreNewAuction(id, bid, robot, pos->GetWorld()->SimTimeNow(), true, frontier);
            }
        }
    }

    void Coordination::CheckAuctions()
    {
        // iterate through all auctions
        for(i_a=auctions.begin(); i_a<auctions.end(); ++i_a){
            // check if auction is open and timeout expired
            if(i_a->open && pos->GetWorld()->SimTimeNow()-i_a->time > TO_AUCTION){
                // close auction
                i_a->open = false;

                // i won, move to frontier
                if(i_a->winner == robot->GetId())
                    robot->Move(i_a->pose);
            }
        }
    }

    void Coordination::StoreNewAuction(int id, double bid, int winner, usec_t time, bool open, Pose pose)
    {
        // invalid bid
        if(bid == BID_INV){
            printf("invalid bid\n");
            return;
        }

        auction_t auction;
        auction.id = id;
        auction.highest_bid = bid;
        auction.winner = winner;
        auction.time = time;
        auction.open = open;
        auction.pose = pose;
        auctions.push_back(auction);
    }

    void Coordination::BroadcastAuction(int id)
    {
        for(i_a=auctions.begin(); i_a<auctions.end(); ++i_a){
            if(i_a->id == id){
                WifiMessageFrontierAuction* msg = new WifiMessageFrontierAuction(id, robot->GetId(), i_a->highest_bid, i_a->pose);
                WifiMessageBase* base_ptr = msg;
                wifi->comm.SendBroadcastMessage(base_ptr);
                break;
            }
        }

        // auction not found
        if(i_a == auctions.end()){
            printf("Could not participate in auction %d because it was not found!\n", id);
        }
    }

    void Coordination::BroadcastBeacon()
    {
        if(to_beacon <= pos->GetWorld()->SimTimeNow()){
            WifiMessageRobot* msg = new WifiMessageRobot(robot->GetId(), robot->GetState(), pos->GetPose());
            WifiMessageBase* base_ptr = msg;
            wifi->comm.SendBroadcastMessage(base_ptr);
            to_beacon = pos->GetWorld()->SimTimeNow() + TO_BEACON;
        }
    }

    int Coordination::WifiUpdate(ModelWifi* wifi, Coordination* cord)
    {
        // visualize wifi connections
        wifi->DataVisualize(cord->robot->GetCam());

        // check if auctions expired and notify winner
        cord->CheckAuctions();

        // send beacon
        cord->BroadcastBeacon();

        return 0; // run again
    }

    void Coordination::ProcessMessage(WifiMessageBase* incoming, void* coordination)
    {
        WifiMessage* msg = dynamic_cast<WifiMessage*>(incoming);
        Coordination* cord = static_cast<Coordination*>(coordination);
        //cout << "received message:" << endl << msg->ToString() << endl;

        switch(msg->type){
            case MSG_ROBOT:
                cord->UpdateRobots(msg->id_robot, msg->state_robot, msg->pose);
                break;

            case MSG_DS:
                printf("got ds message\n");
                break;

            case MSG_FRONTIER_AUCTION:
                // update auction information
                cord->UpdateAuction(msg->id_auction, msg->id_robot, msg->bid, msg->pose);
                break;

            case MSG_DS_AUCTION:
                printf("got ds auction message\n");
                break;

            case MSG_MAP:
                cord->UpdateMap(msg->map);
                break;

            default:
                printf("unknown message type: %d\n", msg->type);
        }

        delete incoming;
    }
}
