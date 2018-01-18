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
                robot->Dock(GetDs(ds_queue.ds_id));
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
            robot->DockQueue(GetDs(ds_queue.ds_id));
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
                // robot is queueing and is at docking station
                if(Queueing()){
                    // one less ahead of me
                    --ds_queue.ahead;
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
            case POL_EVO:
                new_ds = EvoDs(range);
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

    string Coordination::GetWifiModel()
    {
        return wifi->GetConfig()->GetModelString();
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
    
    Ds* Coordination::EvoDs(double range)
    {
        // TODO
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

    int Coordination::WifiUpdate(ModelWifi* wifi, Coordination* cord)
    {
        // visualize wifi connections
        wifi->DataVisualize(cord->robot->GetCam());
        
        // inform other robots about me
        cord->BroadcastBeacon();

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
