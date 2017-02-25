#include "eae_coordination.hh"

using namespace Stg;
using namespace std;

namespace eae
{
    Coordination::Coordination(ModelPosition* pos, Robot* robot)
    {
        // position model to access wifi
        this->pos = pos;

        // read coordination strategy from world file
        Worldfile* wf = pos->GetWorld()->GetWorldFile();
        strategy = (cord_t)wf->ReadInt(0, "strategy", strategy);

        // read docking station selection policy from world file
        policy = (pol_t)wf->ReadInt(0, "policy", policy);

        // initialize wifi adapter
        wifi = (ModelWifi*)pos->GetChild("wifi:0");
        wifi->AddCallback(Model::CB_UPDATE, (model_callback_t)WifiUpdate, this);
        wifi->comm.SetReceiveMsgFn(ProcessMessage, this);
        wifi->Subscribe();

        // robot owning this object
        this->robot = robot;

        // beacon timeout
        to_beacon = 0;

        // statistics
        msgs_sent = 0;
        msgs_received = 0;
        bytes_sent = 0;
        bytes_received = 0;
    }

    void Coordination::Recharge()
    {
        switch(strategy){
            case CORD_MARKET:
                // try to start new docking station auction
                if(DockingAuction() == false)
                    // not enough time between auctions, wait
                    robot->DockQueue(robot->GetDs(), BID_INV);
                break;

            case CORD_GREEDY:
                // send queue message and move to docking station
                DockingQueue();
                break;

            default:
                printf("[%s:%d] [robot %d]: invalid coordination strategy: %d\n", StripPath(__FILE__), __LINE__, robot->GetId(), strategy);
        }
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

    bool Coordination::DockingAuction()
    {
        int ds = robot->GetDs()->id;

        // check if enough time between auctions
        if(ds_auctions.size() > 0){
            vector<ds_auction_t>::iterator it;
            for(it=ds_auctions.end()-1; it>=ds_auctions.begin(); --it){
                // auction for same docking station
                if(it->ds_id == ds){
                    // not enough time between auctions
                    if(pos->GetWorld()->SimTimeNow() < it->time + TO_NEXT_AUCTION){
                        return false;
                    }

                    // enough time between auctions
                    break;
                }
            }
        }

        // increment auction id
        int id = ++auction_id;

        // calculate bid
        double bid = DockingBid(ds);

        if(InArray(this->robot->GetId(), DEBUG_ROBOTS, sizeof(DEBUG_ROBOTS)/sizeof(this->robot->GetId())))
            printf("[%s:%d] [robot %d]: start docking auction %d, bid %.2f\n", StripPath(__FILE__), __LINE__, this->robot->GetId(), id, bid);

        // create and store auction
        StoreNewDsAuction(id, bid, robot->GetId(), robot->GetId(), pos->GetWorld()->SimTimeNow(), true, ds);

        // notify other robots
        BroadcastDsAuction(id);

        // started auction successfully
        return true;
    }

    void Coordination::DockingQueue()
    {
        // robot has valid queue and is waiting at docking station
        if(Queueing()){
            // first time, robot should be at docking station now
            if(ds_queue.time == 0){
                // notify other robots
                BroadcastDsQueue();

                // set timer
                ds_queue.time = pos->GetWorld()->SimTimeNow();

                return;
            }

            // not enough time for other robots to respond
            if(pos->GetWorld()->SimTimeNow()-ds_queue.time < TO_AUCTION){
                return;
            }

            // i'm the first one now
            if(ds_queue.ahead < 1 && GetDs(ds_queue.ds_id)->state == STATE_VACANT){
                // mark docking station as occupied
                DsOccupied(ds_queue.ds_id);

                // go charging
                robot->Dock(GetDs(ds_queue.ds_id), BID_INV);
            }
        }

        // set queue data
        else{
            ds_queue.ds_id = robot->GetDs()->id;
            ds_queue.time = 0;
            ds_queue.ahead = 0;

            if(InArray(this->robot->GetId(), DEBUG_ROBOTS, sizeof(DEBUG_ROBOTS)/sizeof(this->robot->GetId())))
                printf("[%s:%d] [robot %d]: in queue for ds %d\n", StripPath(__FILE__), __LINE__, this->robot->GetId(), ds_queue.ds_id);

            // move to docking station
            robot->DockQueue(GetDs(ds_queue.ds_id), BID_INV);
        }
    }

    void Coordination::BroadcastMap()
    {
        BroadcastMap(robot->GetMap());
    }

    void Coordination::BroadcastMap(GridMap* local)
    {
        // create and send message
        WifiMessageMap* msg = new WifiMessageMap(local);
        WifiMessageBase* base_ptr = msg;
        wifi->comm.SendBroadcastMessage(base_ptr);
        delete msg;

        // statistics
        ++msgs_sent;
        bytes_sent += local->Size();
    }

    double Coordination::DistRobot(Pose pose)
    {
        double dist = 0;
        double dist_temp;

        // iterator
        vector<fr_auction_t>::iterator it;

        // closest distance to any frontier that has been auctioned/won by another robot
        for(it=fr_auctions.begin(); it<fr_auctions.end(); ++it){
            if(it->winner != robot->GetId()){
                dist_temp = pose.Distance(it->pose); // euclidean distance to speed it up
                if(dist_temp < dist || dist == 0)
                    dist = dist_temp;
            }

        }

        return dist;
    }

    void Coordination::UpdateDs(int id, Pose pose, ds_state_t state, int change)
    {
        // invalid docking station
        if(id <= 0)
            return;

        // get docking station object from vector
        Ds* ds = GetDs(id);

        // docking station already in vector
        if(ds){
            // update state
            if(state != STATE_UNDEFINED_DS){
                if(InArray(this->robot->GetId(), DEBUG_ROBOTS, sizeof(DEBUG_ROBOTS)/sizeof(this->robot->GetId())))
                    printf("[%s:%d] [robot %d]: ds state %d\n", StripPath(__FILE__), __LINE__, robot->GetId(), state);

                ds->state = state;

            }

            // update number of robots
            if(change != 0){
                ds->robots += change;
                if(ds->robots > robots.size()) // just in case
                    ds->robots = robots.size();
                if(ds->robots < 0) // just in case
                    ds->robots = 0;
            }

            // docking station became vacant
            if(state == STATE_VACANT){
                switch(strategy){
                    case CORD_MARKET:{
                        // currently queueing at that docking station
                        Ds* dsq = robot->Queueing();
                        if(dsq && dsq->id == id){
                            // start new auction
                            Recharge();
                        }
                        break;
                    }

                    case CORD_GREEDY:
                        // robot is queueing and is at docking station
                        if(Queueing()){
                            // one less ahead of me
                            --ds_queue.ahead;
                        }
                        break;

                    default:
                        printf("[%s:%d] [robot %d]: invalid coordination strategy: %d\n", StripPath(__FILE__), __LINE__, robot->GetId(), strategy);
                }
            }
        }

        // new docking station
        else{
            if(InArray(robot->GetId(), DEBUG_ROBOTS, sizeof(DEBUG_ROBOTS)/sizeof(robot->GetId())))
                printf("[%s:%d] [robot %d]: add docking station %d (%.0f,%.0f)\n", StripPath(__FILE__), __LINE__, robot->GetId(), id, pose.x, pose.y);

            // create new docking station object
            ds = new Ds();

            // get model of docking station
            stringstream ds_name;
            ds_name << "ds" << id;
            Model* ds_model = pos->GetWorld()->GetModel(ds_name.str());

            // add new docking station
            ds->id = id;
            if(state != STATE_UNDEFINED_DS){ // new docking stations should be vacant
                ds->state = state;
            }
            ds->pose = pose;
            ds->model = ds_model;
            ds->robots = max(0, change);
            dss.push_back(ds);

            // broadcast docking station information (don't retransmit change)
            WifiMessageDs* msg = new WifiMessageDs(id, state, pose);
            WifiMessageBase* base_ptr = msg;
            wifi->comm.SendBroadcastMessage(base_ptr);

            // statistics
            ++msgs_sent;
            bytes_sent += sizeof(id) + sizeof(state) + sizeof(pose) + sizeof(int);

            // try to continue exploration
            if(robot->GetState() == STATE_FINISHED){
                robot->Continue();
            }

            // start exploration
            else if(robot->GetState() == STATE_IDLE){
                robot->Explore();
            }
        }
    }

    Ds* Coordination::SelectDs(double range, pol_t policy, int exclude, bool end)
    {
        // select global policy
        if(policy == POL_UNDEFINED)
            policy = this->policy;

        // docking station objects
        Ds* old_ds = robot->GetDs();
        Ds* new_ds;

        // select docking station based on policy
        switch(policy){
            case POL_CLOSEST:
                new_ds = ClosestDs();
                break;
            case POL_VACANT:
                new_ds = VacantDs(range);
                break;
            case POL_OPPORTUNE:
                new_ds = OpportuneDs(range, exclude, end);
                break;
            case POL_CURRENT:
                new_ds = CurrentDs(range);
                break;
            case POL_LONELY:
                new_ds = LonelyDs(range);
                break;
            case POL_CONNECTED:
                new_ds = ConnectedDs(range);
                break;
            case POL_COMBINED:
                new_ds = CombinedDs(range);
                break;
            default:
                new_ds = NULL;
                printf("[%s:%d] [robot %d]: invalid policy\n", StripPath(__FILE__), __LINE__, robot->GetId());
        }

        // selected invalid docking station
        if(!new_ds){
            return old_ds;
        }

        // changed docking station
        if(old_ds != new_ds){

            // messages to inform other robots
            WifiMessageDs* msg;
            WifiMessageBase* base_ptr;

            // update old docking station
            if(old_ds){
                if(InArray(this->robot->GetId(), DEBUG_ROBOTS, sizeof(DEBUG_ROBOTS)/sizeof(this->robot->GetId())))
                    printf("[%s:%d] [robot %d]: switched ds: %d (%d robots) -> %d (%d robots)\n", StripPath(__FILE__), __LINE__, this->robot->GetId(), old_ds->id, old_ds->robots, new_ds->id, new_ds->robots);

                // decrease number at old docking station
                --old_ds->robots;
                if(old_ds->robots < 0) // just in case
                    old_ds->robots = 0;

                // inform other robots about old docking station
                msg = new WifiMessageDs(old_ds->id, old_ds->state, old_ds->pose, -1);
                base_ptr = msg;
                wifi->comm.SendBroadcastMessage(base_ptr);
                delete msg;

                // statistics
                ++msgs_sent;
                bytes_sent += sizeof(int) + sizeof(ds_state_t) + sizeof(Pose) + sizeof(int);
            }
            else if(InArray(this->robot->GetId(), DEBUG_ROBOTS, sizeof(DEBUG_ROBOTS)/sizeof(this->robot->GetId())))
                    printf("[%s:%d] [robot %d]: switched ds: %d (%d robots) -> %d (%d robots)\n", StripPath(__FILE__), __LINE__, this->robot->GetId(), 0, 0, new_ds->id, new_ds->robots);

            // increase number at new docking station
            ++new_ds->robots;
            if(new_ds->robots > robots.size()) // just in case
                new_ds->robots = robots.size();

            // inform other robots about new docking station
            msg = new WifiMessageDs(new_ds->id, new_ds->state, new_ds->pose, 1);
            base_ptr = msg;
            wifi->comm.SendBroadcastMessage(base_ptr);
            delete msg;

            // statistics
            ++msgs_sent;
            bytes_sent += sizeof(int) + sizeof(ds_state_t) + sizeof(Pose) + sizeof(int);
        }

        return new_ds;
    }

    void Coordination::DsVacant(int id)
    {
        Ds* ds = GetDs(id);
        if(ds){
            if(InArray(this->robot->GetId(), DEBUG_ROBOTS, sizeof(DEBUG_ROBOTS)/sizeof(this->robot->GetId())))
                printf("[%s:%d] [robot %d]: ds state %d\n", StripPath(__FILE__), __LINE__, robot->GetId(), STATE_VACANT);

            // set local docking station state
            ds->state = STATE_VACANT;

            // invalidate queue
            ds_queue.ds_id = -1;

            // inform other robots
            WifiMessageDs* msg = new WifiMessageDs(id, STATE_VACANT, ds->pose);
            WifiMessageBase* base_ptr = msg;
            wifi->comm.SendBroadcastMessage(base_ptr);
            delete msg;

            // statistics
            ++msgs_sent;
            bytes_sent += sizeof(id) + sizeof(STATE_VACANT) + sizeof(Pose) + sizeof(int);
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

    bool Coordination::OldFrontier(Pose frontier)
    {
        // iterator
        vector<fr_auction_t>::iterator it;

        // iterate through all auctions
        for(it=fr_auctions.begin(); it<fr_auctions.end(); ++it){
            // return true if frontier is in the list of frontier auctions
            if(robot->SamePoint(frontier, it->pose)){
                return true;
            }
        }

        return false;
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

    pol_t Coordination::GetPolicy()
    {
        return policy;
    }

    string Coordination::GetPolicyString()
    {
        return POL_STRING[policy];
    }

    Ds* Coordination::ClosestDs()
    {
        Ds* ds_free = NULL;
        Ds* ds_occ = NULL;
        double dist_free = 0;
        double dist_occ = 0;
        double dist_temp;
        vector<Ds*>::iterator it;

        // iterate over all docking stations to find closest vacant / occupied
        for(it=dss.begin(); it<dss.end(); ++it){
            // compute path distance
            dist_temp = robot->Distance((*it)->pose.x, (*it)->pose.y);

            // could not compute distance, take euclidean distance
            if(dist_temp < 0)
                dist_temp = (*it)->pose.Distance(robot->GetPose());

            // found vacant docking station
            if((*it)->state != STATE_OCCUPIED && (dist_temp < dist_free || dist_free == 0)){
                ds_free = *it;
                dist_free = dist_temp;
            }

            // found occupied docking station
            else if((*it)->state == STATE_OCCUPIED && (dist_temp < dist_occ || dist_occ == 0)){
                ds_occ = *it;
                dist_occ = dist_temp;
            }
        }

        // return vacant if it is closer than occupied or maximum DS_TOLERANCE further away
        if(dist_free > 0 && dist_occ > 0){
            if(dist_free < dist_occ + DS_TOLERANCE)
                return ds_free;
            return ds_occ;
        }

        // return vacant docking station
        if(dist_free > 0)
            return ds_free;

        // return occupied docking station or NULL
        return ds_occ;
    }

    Ds* Coordination::VacantDs(double range)
    {
        Ds* ds_free = NULL;
        Ds* ds_occ = NULL;
        double dist_free = 0;
        double dist_occ = 0;
        double dist_temp;
        vector<Ds*>::iterator it;

        // iterate over all docking stations to find closest vacant / occupied
        for(it=dss.begin(); it<dss.end(); ++it){
            // compute path distance
            dist_temp = robot->Distance((*it)->pose.x, (*it)->pose.y);

            // could not compute distance, take euclidean distance
            if(dist_temp < 0)
                dist_temp = (*it)->pose.Distance(robot->GetPose());

            // found vacant docking station
            if((*it)->state != STATE_OCCUPIED && (dist_temp < dist_free || dist_free == 0)){
                ds_free = *it;
                dist_free = dist_temp;
            }

            // found occupied docking station
            if((*it)->state == STATE_OCCUPIED && (dist_temp < dist_occ || dist_occ == 0)){
                ds_occ = *it;
                dist_occ = dist_temp;
            }
        }

        // return vacant docking station
        if(0 < dist_free && dist_free < range)
            return ds_free;

        // return occupied docking station or NULL
        return ds_occ;
    }

    Ds* Coordination::OpportuneDs(double range, int exclude, bool end)
    {
        bool frontiers; // true if there are frontiers in range of the docking station
        bool reachable; // true if one docking station is reachable by another
        Ds* ds = NULL;
        double dist = 0;
        double dist_temp;
        vector<Ds*> dss_reachable; // docking stations in range of robot
        vector<Ds*> dss_frontiers; // docking stations that have frontiers in range
        vector<Ds*> dss_reach_front; // docking stations in range of robot with frontiers in range
        vector<Ds*> dss_else; // docking stations not in any of the above vectors
        vector<Ds*>::iterator it, jt;

        // iterate over all docking stations and sort into different vectors
        for(it=dss.begin(); it<dss.end(); ++it){
            // exclude docking station
            if((*it)->id == exclude)
                continue;

            // compute path distance
            dist_temp = robot->Distance((*it)->pose.x, (*it)->pose.y);

            // check for reachable frontiers
            frontiers = (robot->FrontiersReachable((*it)->pose, robot->MaxDist(), true).empty() == false);

            // could not compute distance, take euclidean distance
            if(dist_temp < 0)
                dist_temp = (*it)->pose.Distance(robot->GetPose());

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

            // other docking stations
            else{
                dss_else.push_back(*it);
            }
        }

        if(InArray(this->robot->GetId(), DEBUG_ROBOTS, sizeof(DEBUG_ROBOTS)/sizeof(this->robot->GetId())))
            printf("[%s:%d] [robot %d]: dss_reach_front: %lu, dss_reachable: %lu, dss_frontiers: %lu, dss_else: %lu\n", StripPath(__FILE__), __LINE__, this->robot->GetId(), dss_reach_front.size(), dss_reachable.size(), dss_frontiers.size(), dss_else.size());

        // return closest docking station in range of robot with frontiers in range
        for(it=dss_reach_front.begin(); it<dss_reach_front.end(); ++it){
            // compute path distance
            dist_temp = robot->Distance((*it)->pose.x, (*it)->pose.y);

            // could not compute distance, take euclidean distance
            if(dist_temp < 0)
                dist_temp = (*it)->pose.Distance(robot->GetPose());

            // found docking station
            if(dist_temp < dist || dist == 0){
                ds = *it;
                dist = dist_temp;
            }
        }
        if(dist > 0)
            return ds;

        // at end of exploration, ds with frontiers is required
        if(end && dss_frontiers.empty())
            return ds;

        // return closest docking station in range of robot that has another docking station in range with frontiers in range
        for(it=dss_reachable.begin(); it<dss_reachable.end(); ++it){
            // check if docking station is in the right direction
            // i.e. it is possible to reach a docking station from there which has frontiers in range
            reachable = false;
            for(jt=dss_frontiers.begin(); jt<dss_frontiers.end(); ++jt){
                // comput path distance to other docking station
                dist_temp = robot->GetMap()->Distance((*jt)->pose.x, (*jt)->pose.y, (*it)->pose.x, (*it)->pose.y);

                // could not compute distance, take euclidean distance
                if(dist_temp < 0)
                    dist_temp = (*jt)->pose.Distance((*it)->pose);

                // check if this docking station is reachable from the current one
                if(dist_temp <= range){
                    reachable = true;
                    break;
                }
            }

            // minimize distance
            if(reachable){
                // compute path distance
                dist_temp = robot->Distance((*it)->pose.x, (*it)->pose.y);

                // could not compute distance, take euclidean distance
                if(dist_temp < 0)
                    dist_temp = (*it)->pose.Distance(robot->GetPose());

                // found docking station
                if(dist_temp < dist || dist == 0){
                    ds = *it;
                    dist = dist_temp;
                }
            }
        }
        if(dist > 0)
            return ds;

        // return closest docking station in range of robot
        for(it=dss_reachable.begin(); it<dss_reachable.end(); ++it){
            // compute path distance
            dist_temp = robot->Distance((*it)->pose.x, (*it)->pose.y);

            // could not compute distance, take euclidean distance
            if(dist_temp < 0)
                dist_temp = (*it)->pose.Distance(robot->GetPose());

            // found docking station
            if(dist_temp < dist || dist == 0){
                ds = *it;
                dist = dist_temp;
            }
        }
        if(dist > 0)
            return ds;

        // return closest docking station with frontiers in range
        for(it=dss_frontiers.begin(); it<dss_frontiers.end(); ++it){
            // compute path distance
            dist_temp = robot->Distance((*it)->pose.x, (*it)->pose.y);

            // could not compute distance, take euclidean distance
            if(dist_temp < 0)
                dist_temp = (*it)->pose.Distance(robot->GetPose());

            // found docking station
            if(dist_temp < dist || dist == 0){
                ds = *it;
                dist = dist_temp;
            }
        }
        if(dist > 0)
            return ds;

        // return closest docking station
        for(it=dss_else.begin(); it<dss_else.end(); ++it){
            // compute path distance
            dist_temp = robot->Distance((*it)->pose.x, (*it)->pose.y);

            // could not compute distance, take euclidean distance
            if(dist_temp < 0)
                dist_temp = (*it)->pose.Distance(robot->GetPose());

            // found docking station
            if(dist_temp < dist || dist == 0){
                ds = *it;
                dist = dist_temp;
            }
        }
        return ds; // can return NULL
    }

    Ds* Coordination::CurrentDs(double range)
    {
        // get the current ds of the robot
        Ds* ds_cur = robot->GetDs();

        // return current ds if it still has opportunities
        if(ds_cur && robot->FrontiersReachable(ds_cur->pose, robot->MaxDist(), true).empty() == false)
            return ds_cur;

        // return other ds that still has opportunities
        if(ds_cur)
            return OpportuneDs(range, ds_cur->id);
        else
            return OpportuneDs(range, 0);
    }

    Ds* Coordination::LonelyDs(double range)
    {
        Ds* ds = NULL;
        unsigned int robots = 0;
        double dist = 0;
        double dist_temp;
        vector<Ds*>::iterator it;

        // iterate over all docking stations to find least crowded one
        for(it=dss.begin(); it<dss.end(); ++it){
            // compute path distance
            dist_temp = robot->Distance((*it)->pose.x, (*it)->pose.y);

            // could not compute distance, take euclidean distance
            if(dist_temp < 0)
                dist_temp = (*it)->pose.Distance(robot->GetPose());

            // cannot reach docking station
            if(dist_temp > range)
                continue;

            // found docking station with same number of robots that is closer by
            // or found docking station with fewer robots
            // or it's the first docking station in range
            if(((*it)->robots == robots && dist_temp < dist) || (*it)->robots < robots || !ds){
                ds = *it;
                robots = (*it)->robots;
                dist = dist_temp;
            }
        }

        // return docking station or NULL
        return ds;
    }

    Ds* Coordination::ConnectedDs(double range)
    {
        Ds* ds = NULL;
        double dist = 0;
        double dist_temp;
        vector<Ds*>::iterator it;
        vector<robot_t>::iterator jt;

        // iterate over all docking stations
        for(it=dss.begin(); it<dss.end(); ++it){
            // compute path distance
            dist_temp = robot->Distance((*it)->pose.x, (*it)->pose.y);

            // could not compute distance, take euclidean distance
            if(dist_temp < 0)
                dist_temp = (*it)->pose.Distance(robot->GetPose());

            // cannot reach docking station
            if(dist_temp > range)
                continue;

            // count robots still connected to after moving to docking stations
            int connections = 0;
            for(jt=robots.begin(); jt<robots.end(); ++jt){
                if(jt->pose.Distance((*it)->pose) <= wifi->GetConfig()->GetRange())
                    ++connections;
            }

            // all connections could be lost when moving to docking station, skip it
            if(connections <= 0)
                continue;

            // found docking station
            if(dist_temp < dist || !ds){
                ds = *it;
                dist = dist_temp;
            }
        }

        // return docking station or NULL
        return ds;
    }

    Ds* Coordination::CombinedDs(double range)
    {
        /**
         * data structures
         *   vector for last x ds poses TODO
         *
         * policy
         *   combine: closest, opportune, lonely, flocking
         *
         * procedure
         *   if still frontiers
         *     select current ds
         *   else
         *     compute geometric mean of current ds each robot selected
         *     compute mean of moving directions of all robots
         *     select ds according to policy
         *       init: no direction, first robots selects ds and sets direction
         *       else: select ds in mean direction +- x° TODO
         */
        return new Ds();
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

        // try to continue exploration
        if(this->robot->GetState() == STATE_FINISHED){
            this->robot->Continue();
        }

        // robot is not active
        if(this->robot->GetState() == STATE_UNDEFINED_ROBOT || this->robot->GetState() == STATE_INIT || this->robot->GetState() == STATE_DEAD)
            return;

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
        Ds* ds_dock = this->robot->Docking();
        if(ds_dock){
            StoreNewFrAuction(id, bid, -1, robot, pos->GetWorld()->SimTimeNow(), false, frontier);
            return;
        }

        // auction not found
        if(it == fr_auctions.end()){
            // calculate bid
            double my_bid = this->robot->CalcBid(frontier);

            // invalid bid, robot cannot reach frontier
            if(my_bid == BID_INV){
                StoreNewFrAuction(id, bid, -1, robot, pos->GetWorld()->SimTimeNow(), false, frontier);
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

        // robot is not active
        if(this->robot->GetState() == STATE_UNDEFINED_ROBOT || this->robot->GetState() == STATE_INIT || this->robot->GetState() == STATE_DEAD || this->robot->GetState() == STATE_FINISHED)
            return;

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
        Ds* ds_dock = this->robot->Docking();
        if(ds_dock && ds_dock->id != ds)
            return;

        // for closest policy, only respond to auctions for closest docking station
        if(policy == POL_CLOSEST){
            if(ds != ClosestDs()->id)
                return;
        }

        // respond to auction
        // calculate bid
        double my_bid = DockingBid(ds);

        // invalid bid
        if(my_bid == BID_INV){
            printf("[%s:%d] [robot %d]: invalid bid\n", StripPath(__FILE__), __LINE__, this->robot->GetId());
            return;
        }

        // update auction information
        UpdateDsAuction(id, this->robot->GetId(), ds, my_bid);

        // my bid is higher
        if(my_bid > bid){
            if(InArray(this->robot->GetId(), DEBUG_ROBOTS, sizeof(DEBUG_ROBOTS)/sizeof(this->robot->GetId())))
                printf("[%s:%d] [robot %d]: participate in docking auction %d, bid %.2f\n", StripPath(__FILE__), __LINE__, this->robot->GetId(), id, my_bid);

            // notify other robots
            BroadcastDsAuction(id);
        }
    }

    bool Coordination::CheckFrontierAuctions()
    {
        // iterator
        vector<fr_auction_t>::iterator it;

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

        // got goal, move to frontier
        if(goal.id > 0){
            robot->SetGoal(goal.pose, goal.highest_bid);
            return false; // only dock if no frontier auction was won
        }

        // lost my own auction, continue exploration
        if(lost){
            if(InArray(robot->GetId(), DEBUG_ROBOTS, sizeof(DEBUG_ROBOTS)/sizeof(robot->GetId())))
                printf("[%s:%d] [robot %d]: lost auction\n", StripPath(__FILE__), __LINE__, robot->GetId());
            robot->Explore();
            return false; // only dock if i didn't start any frontier auction
        }

        return true;
    }

    void Coordination::CheckDsAuctions()
    {
        // iterator
        vector<ds_auction_t>::iterator jt;

        // docking station auctions relevant for this robot
        ds_auction_t ds_dock; // dock at this ds
        ds_dock.id = -1;
        ds_dock.ds_id = -1;
        ds_dock.highest_bid = -1;
        ds_auction_t ds_queue; // queue at this ds
        ds_queue.id = -1;
        ds_queue.ds_id = -1;
        ds_queue.highest_bid = -1;
        ds_auction_t ds_undock; // undock from this ds
        ds_undock.id = -1;

        // iterate through all docking station auctions
        for(jt=ds_auctions.begin(); jt<ds_auctions.end(); ++jt){
            // check if auction is open and timeout expired
            if(jt->open && pos->GetWorld()->SimTimeNow()-jt->time > TO_AUCTION){
                // close auction
                jt->open = false;

                // i won, move to docking station
                if(jt->winner == robot->GetId()){
                    ds_dock = *jt;
                    continue;
                }

                // i lost and am currently charging at that docking station
                // now i have to move away
                Ds* ds = robot->Docking();
                if(ds && ds->id == jt->ds_id){
                    ds_undock = *jt;
                    continue;
                }

                // i lost my own auction, queue at the docking station
                if(jt->initiator == robot->GetId()){
                    ds_queue = *jt;
                    continue;
                }
            }
        }

        // start docking
        if(ds_dock.id > 0){
            // mark docking station as occupied
            DsOccupied(ds_dock.id);

            // start docking procedure
            robot->Dock(GetDs(ds_dock.ds_id), ds_dock.highest_bid);
        }

        // queue at the docking station
        else if(ds_queue.id > 0)
            robot->DockQueue(GetDs(ds_queue.ds_id), ds_queue.highest_bid);

        // undock :(
        if(ds_undock.id > 0){
            // mark docking station as vacant
            DsVacant(ds_undock.id);

            // stop charging, continue exploration
            robot->UnDock();
        }
    }

    bool Coordination::Queueing()
    {
        Ds* dsq = robot->QueueingAtDs();

        // robot is queueing at the docking station stored in queue
        if(dsq && dsq->id == ds_queue.ds_id){
            return true;
        }

        // robot is not queueing
        return false;
    }

    void Coordination::UpdateQueue(int to, int from, int ds)
    {
        // robot is not active
        if(this->robot->GetState() == STATE_UNDEFINED_ROBOT || this->robot->GetState() == STATE_INIT || this->robot->GetState() == STATE_DEAD || this->robot->GetState() == STATE_FINISHED)
            return;

        // this is a response message for me
        // and robot is queueing at docking station
        if(to == robot->GetId() && Queueing()){
                if(InArray(this->robot->GetId(), DEBUG_ROBOTS, sizeof(DEBUG_ROBOTS)/sizeof(this->robot->GetId())))
                    printf("[%s:%d] [robot %d]: robot %d in queue for ds %d ahead of me\n", StripPath(__FILE__), __LINE__, robot->GetId(), from, ds);

                // update queue information
                ++ds_queue.ahead;
            return;
        }

        // this is an initial queueing message
        if(to < 0){
            // possible queueing or charging docking station
            Ds* dsq = robot->Queueing();
            Ds* dsd = robot->Docking();

            // robot is queueing or charging at that docking station
            if((dsq && dsq->id == ds && Queueing()) || (dsd && dsd->id == ds)){
                // send reply
                BroadcastDsQueue(from);
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
                // create and send message
                WifiMessageFrontierAuction* msg = new WifiMessageFrontierAuction(id, robot->GetId(), it->highest_bid, it->pose);
                WifiMessageBase* base_ptr = msg;
                wifi->comm.SendBroadcastMessage(base_ptr);
                delete msg;

                // statistics
                ++msgs_sent;
                bytes_sent += sizeof(id) + sizeof(int) + sizeof(it->highest_bid) + sizeof(it->pose);

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
                // create and send message
                WifiMessageDsAuction* msg = new WifiMessageDsAuction(id, robot->GetId(), it->ds_id, it->highest_bid);
                WifiMessageBase* base_ptr = msg;
                wifi->comm.SendBroadcastMessage(base_ptr);
                delete msg;

                // statistics
                ++msgs_sent;
                bytes_sent += sizeof(id) + sizeof(int) + sizeof(it->ds_id) + sizeof(it->highest_bid);

                break;
            }
        }

        // auction not found
        if(it == ds_auctions.end()){
            printf("[%s:%d] [robot %d]: Could not participate in auction %d because it was not found!\n", StripPath(__FILE__), __LINE__, robot->GetId(), id);
        }
    }

    void Coordination::BroadcastDsQueue(int to)
    {
        // this robot is not queueing
        if(ds_queue.ds_id <= 0)
            return;

        // create and send message
        WifiMessageDsQueue* msg = new WifiMessageDsQueue(to, robot->GetId(), ds_queue.ds_id);
        WifiMessageBase* base_ptr = msg;
        wifi->comm.SendBroadcastMessage(base_ptr);
        delete msg;

        // statistics
        ++msgs_sent;
        bytes_sent += sizeof(to) + sizeof(int) + sizeof(ds_queue.ds_id);
    }

    void Coordination::BroadcastBeacon()
    {
        if(to_beacon <= pos->GetWorld()->SimTimeNow()){
            // create and send message
            WifiMessageRobot* msg = new WifiMessageRobot(robot->GetId(), robot->GetState(), pos->GetPose());
            WifiMessageBase* base_ptr = msg;
            wifi->comm.SendBroadcastMessage(base_ptr);
            delete msg;
            to_beacon = pos->GetWorld()->SimTimeNow() + TO_BEACON;

            // statistics
            ++msgs_sent;
            bytes_sent += sizeof(int) + sizeof(robot_state_t) + sizeof(Pose);
        }
    }

    Ds* Coordination::GetDs(int id)
    {
        vector<Ds*>::iterator it;
        for(it=dss.begin(); it<dss.end(); ++it){
            if((*it)->id == id)
                return *it;
        }
        return NULL;
    }

    ds_state_t Coordination::DsState(int id)
    {
        Ds* ds = GetDs(id);
        if(ds)
            return GetDs(id)->state;
        else
            return STATE_UNDEFINED_DS;
    }

    Pose Coordination::DsPose(int id)
    {
        Ds* ds = GetDs(id);
        if(ds)
            return GetDs(id)->pose;
        else
            return Pose();
    }

    void Coordination::DsOccupied(int id)
    {
        Ds* ds = GetDs(id);
        if(ds){
            if(InArray(this->robot->GetId(), DEBUG_ROBOTS, sizeof(DEBUG_ROBOTS)/sizeof(this->robot->GetId())))
                printf("[%s:%d] [robot %d]: ds state %d\n", StripPath(__FILE__), __LINE__, robot->GetId(), STATE_OCCUPIED);

            // set local docking station state
            ds->state = STATE_OCCUPIED;

            // inform other robots
            WifiMessageDs* msg = new WifiMessageDs(id, STATE_OCCUPIED, ds->pose);
            WifiMessageBase* base_ptr = msg;
            wifi->comm.SendBroadcastMessage(base_ptr);
            delete msg;

            // statistics
            ++msgs_sent;
            bytes_sent += sizeof(id) + sizeof(STATE_OCCUPIED) + sizeof(Pose) + sizeof(int);
        }
    }

    double Coordination::DockingBid(int id)
    {
        double l1, l2, l3, l4;
        vector<Ds*>::iterator itd;
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
            if((*itd)->state != STATE_OCCUPIED)
                ++num_dss;

        // number of active robots, i.e. robots that are willing to charge
        int num_robs = 0;
        for(itr=robots.begin(); itr<robots.end(); ++itr)
            if(itr->state == STATE_IDLE || itr->state == STATE_EXPLORE || itr->state == STATE_CHARGE_QUEUE || itr->state == STATE_PRECHARGE || itr->state == STATE_GOING_CHARGING)
                ++num_robs;

        // check this' robot status
        if(robot->GetState() == STATE_IDLE || robot->GetState() == STATE_EXPLORE || robot->GetState() == STATE_CHARGE_QUEUE || robot->GetState() == STATE_PRECHARGE || robot->GetState() == STATE_GOING_CHARGING)
            ++num_robs;

        // calculate first parameter
        if(num_dss > num_robs) // enough docking stations
            l1 = 1;
        else if(robot->GetState() == STATE_CHARGE) // put a bias when this robot is charging
            l1 = 1;
        else
            l1 = (double)num_dss / (double)num_robs;

        /********************
         * second parameter *
         ********************/

        // charge time
        double time_charge = robot->RemainingChargeTime();

        // remaining run time
        double time_run = robot->RemainingTime();

        // calculate second parameter
        l2 = time_charge / (time_charge + time_run);


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

        // get docking station from id
        Ds* ds = GetDs(id);

        // docking station not found
        if(!ds){
            printf("[%s:%d] [robot %d]: invalid docking station id: %d\n", StripPath(__FILE__), __LINE__, robot->GetId(), id);
            l4 = 0.5;
        }

        // docking station found
        else{
            // distance to docking station
            // compute path distance
            double dist_ds = robot->Distance(ds->pose.x, ds->pose.y);

            // could not compute distance, take euclidean distance
            if(dist_ds < 0)
                dist_ds = robot->GetPose().Distance(ds->pose);

            // get frontiers, try smaller vector first
            vector< vector <int> > frontiers_temp;
            if(frontiers_close.empty())
                frontiers_temp = frontiers;
            else
                frontiers_temp = frontiers_close;

            // distance to closest frontier (job)
            double dist_job = 0;
            double dist_temp;
            if(frontiers_temp.size() > 0){
                for(itf=frontiers_temp.begin(); itf<frontiers_temp.end(); ++itf){
                    // compute path distance
                    dist_temp = robot->Distance(itf->at(0), itf->at(1));

                    // could not compute distance, take euclidean distance
                    if(dist_temp < 0)
                        dist_temp = robot->GetPose().Distance(Pose(itf->at(0), itf->at(1), 0, 0));

                    // found frontier
                    if(dist_temp < dist_job || dist_job == 0){
                        dist_job = dist_temp;
                    }
                }
            }

            // no frontier found
            if(dist_job == 0){
                printf("[%s:%d] [robot %d]: could not comput distance to any frontier\n", StripPath(__FILE__), __LINE__, robot->GetId());
                l4 = 0.5;
            }

            // calculate fourth parameter
            else{
                l4 = dist_job / (dist_job + dist_ds);
            }
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

        // check if auctions expired and take actions
        bool idle = cord->CheckFrontierAuctions();

        // robot uses market based coordination
        if(cord->strategy == CORD_MARKET){
            // robot is not exploring, check docking station auctions
            if(idle)
                cord->CheckDsAuctions();

            // inform other robots about me
            cord->BroadcastBeacon();
        }

        return 0; // run again
    }

    void Coordination::ProcessMessage(WifiMessageBase* incoming, void* coordination)
    {
        WifiMessage* msg = dynamic_cast<WifiMessage*>(incoming);
        Coordination* cord = static_cast<Coordination*>(coordination);

        ++cord->msgs_received;

        switch(msg->type){
            case MSG_ROBOT:
                // add/update robot in the private vector of robots
                cord->UpdateRobots(msg->id_robot, msg->state_robot, msg->pose);
                cord->bytes_received += sizeof(msg->id_robot) + sizeof(msg->state_robot) + sizeof(msg->pose);
                break;

            case MSG_DS:
                // add/update docking station in the private vector of docking stations
                cord->UpdateDs(msg->id_ds, msg->pose, msg->state_ds, msg->change);
                cord->bytes_received += sizeof(msg->id_ds) + sizeof(msg->pose) + sizeof(msg->state_ds) + sizeof(msg->change);
                break;

            case MSG_FRONTIER_AUCTION:
                // update auction information and respond if necessary
                cord->UpdateFrAuction(msg->id_auction, msg->id_robot, msg->bid, msg->pose);
                cord->bytes_received += sizeof(msg->id_auction) + sizeof(msg->id_robot) + sizeof(msg->bid) + sizeof(msg->pose);
                break;

            case MSG_DS_AUCTION:
                // update auction information and respond if necessary
                cord->UpdateDsAuction(msg->id_auction, msg->id_robot, msg->id_ds, msg->bid);
                cord->bytes_received += sizeof(msg->id_auction) + sizeof(msg->id_robot) + sizeof(msg->id_ds) + sizeof(msg->bid);
                break;

            case MSG_DS_QUEUE:
                // update queue information and respond if necessary
                cord->UpdateQueue(msg->id_auction, msg->id_robot, msg->id_ds); // id_auction = id_to
                cord->bytes_received += sizeof(msg->id_auction) + sizeof(msg->id_robot) + sizeof(msg->id_ds);
                break;

            case MSG_MAP:
                // update local map
                cord->UpdateMap(msg->map);
                cord->bytes_received += msg->map->Size();
                break;

            default:
                printf("[%s:%d] [robot %d]: unknown message type: %d\n", StripPath(__FILE__), __LINE__, cord->robot->GetId(), msg->type);
        }

        delete incoming;
    }
}
