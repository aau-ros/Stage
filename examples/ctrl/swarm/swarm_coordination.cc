#include "swarm_coordination.hh"

using namespace Stg;
using namespace std;

namespace swarm
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

        // statistics
        msgs_sent = 0;
        msgs_received = 0;
        bytes_sent = 0;
        bytes_received = 0;
    }

    void Coordination::UpdateDs(int id, Pose pose)
    {
        // invalid docking station
        if(id <= 0)
            return;

        // get docking station object from vector
        Ds* ds = GetDs(id);

        // new docking station
        if(ds == NULL){
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
            ds->pose = pose;
            ds->model = ds_model;
            dss.push_back(ds);

            // broadcast docking station information (don't retransmit change)
            WifiMessageDs* msg = new WifiMessageDs(id, pose);
            WifiMessageBase* base_ptr = msg;
            wifi->comm.SendBroadcastMessage(base_ptr);

            // statistics
            ++msgs_sent;
            bytes_sent += sizeof(id) + sizeof(pose);
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
    
    int Coordination::NumRobots()
    {
        return robots.size();
    }
    
    float Coordination::RobotDensity(radians_t angle_min, radians_t angle_max)
    {
        // no robots there
        if(NumRobots() == 0)
            return 0;
        
        // number of robots in sector
        int num_robots = 0;
        
        // distances to robots in sector
        meters_t distances = 0;
        
        // iterator
        vector<robot_t>::iterator it;
        
        // iterate through all robots
        for(it=robots.begin(); it<robots.end(); ++it){
            // offset between robots
            meters_t dx = it->pose.x - pos->GetPose().x;
            meters_t dy = it->pose.y - pos->GetPose().y;
            
            // angle between robots in global coordinates
            radians_t ag = atan2(dy, dx);
            
            // angle relative to robot heading
            radians_t angle = ag - pos->GetPose().a;
            
            // shift angle to start from negative y axis (of robot's coordinates)
            angle += PI;
            
            // normalize angle to [0,2π]
            if(angle < 0)
                angle += 2*PI;
            if(2*PI <= angle)
                angle -= 2*PI;
            
            // robots in given sector
            if(angle_min <= angle && angle < angle_max){
                // count robots
                ++num_robots;
                
                // distance between robots
                meters_t dist = hypot(dx, dy);
                
                // clip distance within reasonable range
                if(dist < CLOSE_DIST)
                    dist = 0;
                if(dist > wifi->GetConfig()->GetRange())
                    dist = wifi->GetConfig()->GetRange();
                
                // add distance between robots
                distances += dist;
            }
        }
        
        // robot density in sector
        float density = (float)num_robots / (float)NumRobots();
        
        // normalize density with average distances
        if(num_robots > 0)
            density *= 1 - (float)distances / (num_robots * (float)wifi->GetConfig()->GetRange());
        else
            density = 0;
        
        return density;
    }

    string Coordination::GetWifiModel()
    {
        return wifi->GetConfig()->GetModelString();
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
                it->update = pos->GetWorld()->SimTimeNow();
                break;
            }
        }

        // robot not found
        if(it == robots.end()){
            if(InArray(robot->GetId(), DEBUG_ROBOTS, sizeof(DEBUG_ROBOTS)/sizeof(robot->GetId())))
                printf("[%s:%d] [robot %d]: add robot %d (%.0f,%.0f)\n", StripPath(__FILE__), __LINE__, robot->GetId(), id, pose.x, pose.y);
                
            // insert new robot into list
            robot_t robot;
            robot.id = id;
            robot.state = state;
            robot.pose = pose;
            robot.update = pos->GetWorld()->SimTimeNow();
            robots.push_back(robot);
        }
    }

    void Coordination::UpdateRobots()
    {
        // iterator
        vector<robot_t>::iterator it;

        // iterate through all robots
        for(it=robots.begin(); it<robots.end();){
            // robot information is out of date
            if(it->update + 2 * TO_BEACON < pos->GetWorld()->SimTimeNow()){
                if(InArray(robot->GetId(), DEBUG_ROBOTS, sizeof(DEBUG_ROBOTS)/sizeof(robot->GetId())))
                    printf("[%s:%d] [robot %d]: remove robot %d\n", StripPath(__FILE__), __LINE__, robot->GetId(), it->id);
                
                it = robots.erase(it);
            }
            else
                ++it;
        }
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

    Pose Coordination::DsPose(int id)
    {
        Ds* ds = GetDs(id);
        if(ds)
            return GetDs(id)->pose;
        else
            return Pose();
    }

    int Coordination::WifiUpdate(ModelWifi* wifi, Coordination* cord)
    {
        // visualize wifi connections
        wifi->DataVisualize(cord->robot->GetCam());
        
        // inform other robots about me
        cord->BroadcastBeacon();
        
        // update information about other robots
        cord->UpdateRobots();

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
                cord->UpdateDs(msg->id_ds, msg->pose);
                cord->bytes_received += sizeof(msg->id_ds) + sizeof(msg->pose);
                break;

            default:
                printf("[%s:%d] [robot %d]: unknown message type: %d\n", StripPath(__FILE__), __LINE__, cord->robot->GetId(), msg->type);
        }

        delete incoming;
    }
}
