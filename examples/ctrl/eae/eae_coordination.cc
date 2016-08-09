#include "eae_coordination.hh"

using namespace Stg;
using namespace std;

namespace eae
{
    Coordination::Coordination(ModelPosition* pos, Robot* robot)
    {
        // position model to access wifi
        this->pos = pos;

        // read exploration strategy from world file
        Worldfile* wf = pos->GetWorld()->GetWorldFile();
        strategy = (cord_t)wf->ReadInt(0, "coordination", strategy);

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
            printf("[%s:%d] [robot %d]: invalid bid\n", StripPath(__FILE__), __LINE__, robot->GetId());
            return;
        }

        // increment auction id
        int id = ++auction_id;

        // create and store auction
        StoreNewFrAuction(id, bid, robot->GetId(), robot->GetId(), pos->GetWorld()->SimTimeNow(), true, frontier);

        // notify other robots
        BroadcastFrAuction(id);
    }

    void Coordination::DockingAuction(Pose pose, int ds)
    {
        // check if enough time between auctions
        if(ds_auctions.size() > 0){
            vector<ds_auction_t>::iterator it;
            for(it=ds_auctions.end()-1; it>=ds_auctions.begin(); --it){
                // i started auction for same docking station
                if(it->initiator == robot->GetId() && it->ds_id == ds){
                    // not enough time between auctions
                    if(pos->GetWorld()->SimTimeNow() < it->time + TO_NEXT_AUCTION){
                        return;
                    }

                    // enough time between auctions
                    break;
                }
            }
        }

        // increment auction id
        int id = ++auction_id;

        // calculate bid
        double bid = BID_INV;
        if(strategy == CORD_MARKET)
            bid = DockingBid(ds);
        else if(strategy == CORD_GREEDY)
            bid = BID_MAX;
        else
            printf("[%s:%d] [robot %d]: invalid coordination strategy: %d\n", StripPath(__FILE__), __LINE__, this->robot->GetId(), strategy);

        // create and store auction
        StoreNewDsAuction(id, bid, robot->GetId(), robot->GetId(), pos->GetWorld()->SimTimeNow(), true, ds);

        // notify other robots
        BroadcastDsAuction(id);
    }

    void Coordination::BroadcastMap()
    {
        BroadcastMap(robot->GetMap());
    }

    void Coordination::BroadcastMap(GridMap* local)
    {
        WifiMessageMap* msg = new WifiMessageMap(local);
        WifiMessageBase* base_ptr = msg;
        wifi->comm.SendBroadcastMessage(base_ptr);
    }

    double Coordination::DistRobot(Pose pose)
    {
        double dist = 0;
        double dist_temp;
        vector<robot_t>::iterator it;
        for(it=robots.begin(); it<robots.end(); ++it){
            dist_temp = robot->Distance(it->pose.x, it->pose.y);
            if(dist_temp < dist || dist == 0)
                dist = dist_temp;
        }
        return dist;
    }

    void Coordination::UpdateDs(int id, Pose pose, ds_state_t state)
    {
        // check if docking station is already in vector
        vector<ds_t>::iterator it;
        for(it=dss.begin(); it<dss.end(); ++it){
            // docking station already in vector
            if(it->id == id){
                // update state
                it->state = state;

                // start new auction if currently queueing at that docking station
                ds_t ds;
                if(robot->Queueing(ds)){
                    if(ds.id == id && state == STATE_VACANT){
                        DockingAuction(robot->GetPose(), id);
                    }
                }
                return;
            }
        }

        // get model of docking station
        stringstream ds_name;
        ds_name << "ds" << id;
        Model* ds_model = pos->GetWorld()->GetModel(ds_name.str());

        // add new docking station
        ds_t ds;
        ds.id = id;
        ds.state = state;
        ds.pose = pose;
        ds.model = ds_model;
        dss.push_back(ds);

        // broadcast docking station information
        WifiMessageDs* msg = new WifiMessageDs(id, state, pose);
        WifiMessageBase* base_ptr = msg;
        wifi->comm.SendBroadcastMessage(base_ptr);
    }

    ds_t Coordination::SelectDs(double range, pol_t policy)
    {
        switch(policy){
            case POL_CLOSEST:
                return ClosestDs();
                break;
            case POL_VACANT:
                return VacantDs(range);
                break;
            case POL_OPPORTUNISTIC:
                return OpportunisticDs(range);
                break;
            default:
                printf("[%s:%d] [robot %d]: combined policy not yet implemented\n", StripPath(__FILE__), __LINE__, robot->GetId());
                return ds_t();
        }
    }

    void Coordination::DsVacant(int id)
    {
        vector<ds_t>::iterator it;
        for(it=dss.begin(); it<dss.end(); ++it){
            if(it->id == id){
                // set local state
                it->state = STATE_VACANT;

                // inform other robots
                WifiMessageDs* msg = new WifiMessageDs(id, STATE_VACANT, DsPose(id));
                WifiMessageBase* base_ptr = msg;
                wifi->comm.SendBroadcastMessage(base_ptr);

                break;
            }
        }
    }

    bool Coordination::Finished()
    {
        // iterator
        vector<robot_t>::iterator it;

        // iterate through all robots
        for(it=robots.begin(); it<robots.end(); ++it){
            // not all robots are done
            if(it->state != STATE_FINISHED)
                return false;
        }

        // all robots are done
        return true;
    }

    string Coordination::GetWifiModel()
    {
        return wifi->GetConfig()->GetModelString();
    }

    cord_t Coordination::GetStrategy()
    {
        return strategy;
    }

    string Coordination::GetStrategyString()
    {
        return CORD_STRING[strategy];
    }

    ds_t Coordination::ClosestDs()
    {
        ds_t ds_free = ds_t();
        ds_t ds_occ = ds_t();
        double dist_free = 0;
        double dist_occ = 0;
        double dist_temp;
        vector<ds_t>::iterator it;

        // iterate over all docking stations to find closest free / occupied
        for(it=dss.begin(); it<dss.end(); ++it){
            dist_temp = robot->Distance(it->pose.x, it->pose.y);
            if(it->state != STATE_OCCUPIED && (dist_temp < dist_free || dist_free == 0)){
                ds_free = *it;
                dist_free = dist_temp;
            }
            else if(it->state == STATE_OCCUPIED && (dist_temp < dist_occ || dist_occ == 0)){
                ds_occ = *it;
                dist_occ = dist_temp;
            }
        }

        // return free if it is closer than occupied or maximum DS_TOLERANCE further away
        if(dist_free > 0 && dist_occ > 0){
            if(dist_free < dist_occ + DS_TOLERANCE )
                return ds_free;
            return ds_occ;
        }

        // return free docking station
        if(dist_free > 0)
            return ds_free;

        // return occupied docking station (or invalid one)
        return ds_occ;
    }

    ds_t Coordination::VacantDs(double range)
    {
        ds_t ds = ds_t();
        double dist = 0;
        double dist_temp;
        vector<ds_t>::iterator it;

        // iterate over all docking stations to find closest free in range
        for(it=dss.begin(); it<dss.end(); ++it){
            dist_temp = robot->Distance(it->pose.x, it->pose.y);
            if(it->state != STATE_OCCUPIED && dist_temp <= range && (dist_temp < dist || dist == 0)){
                ds = *it;
                dist = dist_temp;
            }
        }

        return ds;
    }

    ds_t Coordination::OpportunisticDs(double range)
    {
//         printf("[%s:%d] [robot %d]: looking for next closest ds\n", StripPath(__FILE__), __LINE__, robot->GetId());
        bool frontiers; // true if there are frontiers in range of the docking station
        bool reachable; // true if one docking station is reachable by another
        ds_t ds = ds_t();
        double dist = 0;
        double dist_temp;
        vector<ds_t> dss_reachable; // docking stations in range of robot
        vector<ds_t> dss_frontiers; // docking stations that have frontiers in range
        vector<ds_t> dss_reach_front; // docking stations in range of robot with frontiers in range
        vector<ds_t>::iterator it, jt;

        // iterate over all docking stations and sort into different vectors
        for(it=dss.begin(); it<dss.end(); ++it){
            dist_temp = robot->Distance(it->pose.x, it->pose.y);
            frontiers = (robot->FrontiersReachable(it->pose, robot->MaxDist(), true).empty() == false);

            // docking station is reachable and has frontiers in range
            if(dist_temp <= range && frontiers){
                dss_reach_front.push_back(*it);
            }

            // docking station is reachable
            else if(dist_temp <= range){
                dss_reachable.push_back(*it);
            }

            // docking station has frontiers in range
            else if(frontiers){
                dss_frontiers.push_back(*it);
            }
        }

        // return closest docking station in range of robot with frontiers in range
        for(it=dss_reach_front.begin(); it<dss_reach_front.end(); ++it){
            dist_temp = robot->Distance(it->pose.x, it->pose.y);
            if(dist_temp < dist || dist == 0){
                ds = *it;
                dist = dist_temp;
            }
        }
        if(dist > 0)
            return ds;

        // return closest docking station in range of robot
        for(it=dss_reachable.begin(); it<dss_reachable.end(); ++it){
            // check if docking station is in the right direction
            // i.e. it is possible to reach a docking station from there which has frontiers in range
            reachable = false;
            for(jt=dss_frontiers.begin(); jt<dss_frontiers.end(); ++jt){
                if(hypot(jt->pose.x - it->pose.x, jt->pose.y - it->pose.y) <= range){
                    reachable = true;
                    break;
                }
            }

            // minimize distance
            if(reachable){
                dist_temp = robot->Distance(it->pose.x, it->pose.y);
                if(dist_temp < dist || dist == 0){
                    ds = *it;
                    dist = dist_temp;
                }
            }
        }
        return ds;
    }

    void Coordination::UpdateRobots(int id, robot_state_t state, Pose pose)
    {
        //  i don't need to be on the list
        if(id == robot->GetId())
            return;

        // iterator
        vector<robot_t>::iterator it;

        // iterate through all robots
        for(it=robots.begin(); it<robots.end(); ++it){
            // found robot
            if(it->id == id){
                // update state and pose
                it->state = state;
                it->pose = pose;
                break;
            }
        }

        // robot not found
        if(it == robots.end()){
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

    void Coordination::UpdateFrAuction(int id, int robot, double bid, Pose frontier)
    {
        // invalid bid
        if(bid == BID_INV){
            printf("[%s:%d] [robot %d]: invalid bid\n", StripPath(__FILE__), __LINE__, this->robot->GetId());
            return;
        }

        // iterator
        vector<fr_auction_t>::iterator it;

        // iterate through all auctions
        for(it=fr_auctions.begin(); it<fr_auctions.end(); ++it){
            // found auction
            if(it->id == id){
                // check if auction is still open and if bid is higher
                if(it->open && it->highest_bid < bid){
                    it->highest_bid = bid;
                    it->winner = robot;
                }
                break;
            }
        }

        // don't respond to frontier auctions if robot is charging or on the way
        ds_t ds_dock;
        if(this->robot->Docking(ds_dock)){
            return;
        }

        // auction not found
        if(it == fr_auctions.end()){
            // calculate bid
            double my_bid = this->robot->CalcBid(frontier);

            // invalid bid, robot cannot reach frontier
            if(my_bid == BID_INV){
                return;
            }

            // my bid is higher
            if(my_bid > bid){
                // store auction
                StoreNewFrAuction(id, my_bid, -1, this->robot->GetId(), pos->GetWorld()->SimTimeNow(), true, frontier);

                // notify other robots
                BroadcastFrAuction(id);
            }

            // my bid is lower, just store auction
            else{
                StoreNewFrAuction(id, bid, -1, robot, pos->GetWorld()->SimTimeNow(), true, frontier);
            }
        }
    }

    void Coordination::UpdateDsAuction(int id, int robot, int ds, double bid)
    {
        // invalid bid
        if(bid == BID_INV){
            printf("[%s:%d] [robot %d]: invalid bid\n", StripPath(__FILE__), __LINE__, this->robot->GetId());
            return;
        }

        // iterator
        vector<ds_auction_t>::iterator it;

        // iterate through all auctions and update information
        for(it=ds_auctions.begin(); it<ds_auctions.end(); ++it){
            // found auction
            if(it->id == id){
                // check if auction is still open and if bid is higher
                if(it->open && it->highest_bid < bid){
                    it->highest_bid = bid;
                    it->winner = robot;
                }
                return;
            }
        }

        // new auction, store in vector
        StoreNewDsAuction(id, bid, -1, robot, pos->GetWorld()->SimTimeNow(), true, ds);

        // don't respond if just done charging
        if(this->robot->FullyCharged())
            return;

        // don't respond to auctions for other docking stations if robot is charging or on the way
        ds_t ds_dock;
        if(this->robot->Docking(ds_dock)){
            if(ds_dock.id != ds)
                return;
        }

        // don't respond to auctions with greedy strategy
        else if(bid == BID_MAX)
            return;

        // for closest policy, only respond to auctions for closest docking station
        if(POL == POL_CLOSEST){
            if(ds != ClosestDs().id)
                return;
        }

        // respond to auction
        // calculate bid
        double my_bid = BID_INV;
        if(strategy == CORD_MARKET)
            my_bid = DockingBid(ds);
        else if(strategy == CORD_GREEDY)
            my_bid = BID_MAX + 1; // if i'm currently docking, make sure i'm not interrupted
        else
            printf("[%s:%d] [robot %d]: invalid coordination strategy: %d\n", StripPath(__FILE__), __LINE__, this->robot->GetId(), strategy);

        // invalid bid
        if(my_bid == BID_INV){
            printf("[%s:%d] [robot %d]: invalid bid\n", StripPath(__FILE__), __LINE__, this->robot->GetId());
            return;
        }

        // my bid is higher
        if(my_bid > bid){
            // update auction information
            UpdateDsAuction(id, this->robot->GetId(), ds, my_bid);

            // notify other robots
            BroadcastDsAuction(id);
        }
    }

    void Coordination::CheckAuctions()
    {
        // iterators
        vector<fr_auction_t>::iterator it;
        vector<ds_auction_t>::iterator jt;

        // store frontier of won frontier auction
        fr_auction_t goal;
        goal.id = -1;

        // looser of frontier auction
        bool lost = false;

        // iterate through all frontier auctions
        for(it=fr_auctions.begin(); it<fr_auctions.end(); ++it){
            // check if auction is open and timeout expired
            if(it->open && pos->GetWorld()->SimTimeNow()-it->time > TO_AUCTION){
                // close auction
                it->open = false;

                // i won the auction
                if(it->winner == robot->GetId()){
                    // i was the initiator, move to that frontier now
                    if(it->initiator == robot->GetId()){
                        goal = *it;
                        break;
                    }
                    // i was not the initiator
                    // store frontier and keep looking for other won auctions
                    // keep frontier with highest bid
                    if(goal.id > 0){
                        if(goal.highest_bid < it->highest_bid)
                            goal = *it;
                    }
                    else
                        goal = *it;
                }

                // i lost my own auction, continue exploration
                else if(it->initiator == robot->GetId()){
                    lost = true;
                }
            }
        }

        // move to frontier
        if(goal.id > 0){
            robot->SetGoal(goal.pose, goal.highest_bid);
            return; // only dock if no frontier auction was won
        }

        // continue exploration
        if(lost){
            robot->Explore();
            return; // only dock if i didn't start any frontier auction
        }

        // iterate through all docking station auctions
        for(jt=ds_auctions.begin(); jt<ds_auctions.end(); ++jt){
            // check if auction is open and timeout expired
            if(jt->open && pos->GetWorld()->SimTimeNow()-jt->time > TO_AUCTION){
                // close auction
                jt->open = false;

                // mark docking station as occupied
                DsOccupied(jt->ds_id);

                // i won, move to docking station
                if(jt->winner == robot->GetId()){
                    robot->Dock(GetDs(jt->ds_id), jt->highest_bid);
                    continue;
                }

                // i lost and am currently charging at that docking station
                // now i have to move away
                ds_t ds;
                if(robot->Charging(ds)){
                    if(ds.id == jt->ds_id){
                        robot->UnDock();
                        continue;
                    }
                }

                // i lost my own auction, queue at the docking station
                if(jt->initiator == robot->GetId()){
                    robot->DockQueue(GetDs(jt->ds_id), jt->highest_bid);
                }
            }
        }
    }

    void Coordination::StoreNewFrAuction(int id, double bid, int initiator, int winner, usec_t time, bool open, Pose pose)
    {
        // invalid bid
        if(bid == BID_INV){
            printf("[%s:%d] [robot %d]: invalid bid\n", StripPath(__FILE__), __LINE__, robot->GetId());
            return;
        }

        fr_auction_t auction;
        auction.id = id;
        auction.highest_bid = bid;
        auction.initiator = initiator;
        auction.winner = winner;
        auction.time = time;
        auction.open = open;
        auction.pose = pose;
        fr_auctions.push_back(auction);
    }

    void Coordination::StoreNewDsAuction(int id, double bid, int initiator, int winner, usec_t time, bool open, int ds_id)
    {
        // invalid bid
        if(bid == BID_INV){
            printf("[%s:%d] [robot %d]: invalid bid\n", StripPath(__FILE__), __LINE__, robot->GetId());
            return;
        }

        ds_auction_t auction;
        auction.id = id;
        auction.highest_bid = bid;
        auction.initiator = initiator;
        auction.winner = winner;
        auction.time = time;
        auction.open = open;
        auction.ds_id = ds_id;
        ds_auctions.push_back(auction);
    }

    void Coordination::BroadcastFrAuction(int id)
    {
        // iterator
        vector<fr_auction_t>::iterator it;

        // iterate through all auctions
        for(it=fr_auctions.begin(); it<fr_auctions.end(); ++it){
            if(it->id == id){
                WifiMessageFrontierAuction* msg = new WifiMessageFrontierAuction(id, robot->GetId(), it->highest_bid, it->pose);
                WifiMessageBase* base_ptr = msg;
                wifi->comm.SendBroadcastMessage(base_ptr);
                break;
            }
        }

        // auction not found
        if(it == fr_auctions.end()){
            printf("[%s:%d] [robot %d]: Could not participate in auction %d because it was not found!\n", StripPath(__FILE__), __LINE__, robot->GetId(), id);
        }
    }

    void Coordination::BroadcastDsAuction(int id)
    {
        // iterator
        vector<ds_auction_t>::iterator it;

        // iterate through all auctions
        for(it=ds_auctions.begin(); it<ds_auctions.end(); ++it){
            if(it->id == id){
                WifiMessageDsAuction* msg = new WifiMessageDsAuction(id, robot->GetId(), robot->GetState(), it->ds_id, DsState(it->ds_id), it->highest_bid, DsPose(it->ds_id));
                WifiMessageBase* base_ptr = msg;
                wifi->comm.SendBroadcastMessage(base_ptr);
                break;
            }
        }

        // auction not found
        if(it == ds_auctions.end()){
            printf("[%s:%d] [robot %d]: Could not participate in auction %d because it was not found!\n", StripPath(__FILE__), __LINE__, robot->GetId(), id);
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

    ds_t Coordination::GetDs(int id)
    {
        vector<ds_t>::iterator it;
        for(it=dss.begin(); it<dss.end(); ++it){
            if(it->id == id)
                return *it;
        }
        return ds_t();
    }

    ds_state_t Coordination::DsState(int id)
    {
        vector<ds_t>::iterator it;
        for(it=dss.begin(); it<dss.end(); ++it){
            if(it->id == id)
                return it->state;
        }
        return STATE_UNDEFINED_DS;
    }

    Pose Coordination::DsPose(int id)
    {
        vector<ds_t>::iterator it;
        for(it=dss.begin(); it<dss.end(); ++it){
            if(it->id == id)
                return it->pose;
        }
        return Pose();
    }

    void Coordination::DsOccupied(int id)
    {
        vector<ds_t>::iterator it;
        for(it=dss.begin(); it<dss.end(); ++it){
            if(it->id == id){
                it->state = STATE_OCCUPIED;
                break;
            }
        }
    }

    double Coordination::DockingBid(int ds)
    {
        double l1, l2, l3, l4;
        vector<ds_t>::iterator itd;
        vector<robot_t>::iterator itr;
        vector< vector <int> >::iterator itf;
        vector< vector <int> > frontiers = robot->GetMap()->Frontiers();
        vector< vector <int> > frontiers_close = robot->FrontiersReachable();


        /*******************
         * first parameter *
         *******************/

        // number of not occupied docking stations
        int num_dss = 0;
        for(itd=dss.begin(); itd<dss.end(); ++itd)
            if(itd->state !=  STATE_OCCUPIED)
                ++num_dss;

        // number of active robots
        int num_robs = 0;
        for(itr=robots.begin(); itr<robots.end(); ++itr)
            if(itr->state == STATE_EXPLORE || itr->state == STATE_IDLE || itr->state == STATE_PRECHARGE)
                ++num_robs;

        // calculate first parameter
        if(num_dss > num_robs)
            l1 = 1;
        else
            l1 = (double)num_dss / (num_robs + 1); // +1 since this robot is not in the vector

        /********************
         * second parameter *
         ********************/

        // charge time
        double time_charge = robot->RemainingChargeTime();

        // remaining run time
        double time_run = robot->RemainingTime();

        // calculate second parameter
        l2 = (double)time_charge / (time_charge + time_run);


        /*******************
         * third parameter *
         *******************/

        // number of frontiers (jobs)
        int num_jobs = frontiers.size();

        // number of frontiers in range
        int num_jobs_close = frontiers_close.size();

        // calculate third parameter
        if(num_jobs == 0)
            l3 = 1;
        else
            l3 = (num_jobs - num_jobs_close) / (double)num_jobs;


        /********************
         * fourth parameter *
         ********************/

        // distance to docking station
        double dist_ds = 0;
        for(itd=dss.begin(); itd<dss.end(); ++itd){
            if(itd->id == ds){
                dist_ds = robot->Distance(itd->pose.x, itd->pose.y);
                break;
            }
        }

        // docking station not found
        if(itd == dss.end()){
            printf("[%s:%d] [robot %d]: invalid docking station id: %d\n", StripPath(__FILE__), __LINE__, robot->GetId(), ds);
            l4 = 0.5;
        }

        // docking station found
        else{
            // distance to closest frontier (job)
            double dist_job = 0;
            double dist_temp;
            for(itf=frontiers_close.begin(); itf<frontiers_close.end(); ++itf){
                dist_temp = robot->Distance(itf->at(0), itf->at(1));
                if(dist_temp < dist_job || dist_job == 0){
                    dist_job = dist_temp;
                }
            }

            // calculate fourth parameter
            l4 = dist_job / (dist_job + dist_ds);
        }


        /*****************
         * calculate bid *
         *****************/
        return L1*l1 + L2*l2 + L3*l3 + L4*l4;
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

        switch(msg->type){
            case MSG_ROBOT:
                // add/update robot in the private vector of robots
                cord->UpdateRobots(msg->id_robot, msg->state_robot, msg->pose);
                break;

            case MSG_DS:
                // add/update docking station in the private vector of docking stations
                cord->UpdateDs(msg->id_ds, msg->pose, msg->state_ds);
                break;

            case MSG_FRONTIER_AUCTION:
                // update auction information and respond if necessary
                cord->UpdateFrAuction(msg->id_auction, msg->id_robot, msg->bid, msg->pose);
                break;

            case MSG_DS_AUCTION:
                // update auction information and respond if necessary
                cord->UpdateDsAuction(msg->id_auction, msg->id_robot, msg->id_ds, msg->bid);
                break;

            case MSG_MAP:
                // update local map
                cord->UpdateMap(msg->map);
                break;

            default:
                printf("[%s:%d] [robot %d]: unknown message type: %d\n", StripPath(__FILE__), __LINE__, cord->robot->GetId(), msg->type);
        }

        delete incoming;
    }
}
