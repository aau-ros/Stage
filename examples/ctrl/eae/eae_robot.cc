#include "eae_robot.hh"

using namespace Stg;
using namespace std;

namespace eae
{
    Robot::Robot(ModelPosition* pos, int robots, int dss)
    {
        // robot id
        id = pos->GetId();

        // instantiate objects
        this->pos = pos;
        fid = (ModelFiducial*)pos->GetUnusedModelOfType("fiducial");
        map = new GridMap(pos->GetPose(), pos->GetWorld(), id);
        cord = new Coordination(pos, this);
        log = new LogOutput(id, robots, dss, cord->GetWifiModel(), cord->GetStrategy(), cord->GetStrategyString(), pos->FindPowerPack()->GetCapacity());
        cam = new OrthoCamera();
        wpcolor = Color(0,1,0); // waypoint color is green

        // robot still needs to be initialized
        state = STATE_INIT;

        // register callback for position updates
        pos->AddCallback(Model::CB_UPDATE, (model_callback_t)PositionUpdate, this);
        pos->Subscribe();

        // register callback for fiducial updates
        fid->AddCallback(Model::CB_UPDATE, (model_callback_t)FiducialUpdate, this);
        fid->Subscribe();

        // store starting point for visualization
        pos->waypoints.push_back(ModelPosition::Waypoint(pos->GetPose(), wpcolor));

        // distance traveled
        dist_travel = 0;

        // goal queue is empty
        goal_next_bid = BID_INV;

        // charging power
        charging_watt = 0;

        // last charging time
        last_charge = 0;
    }

    void Robot::Init()
    {
        // do an initial map update
        UpdateMap();

        // pose at last map update
        last_pose = pos->GetPose();

        // init done
        state = STATE_IDLE;
    }

    void Robot::Explore()
    {
        // only explore if currently exploring or idle
        if(state != STATE_EXPLORE){
            if(state != STATE_IDLE)
                return;
            state = STATE_EXPLORE;
        }

        // get current position
        Pose pose = pos->GetPose();


        /******************
         * determine goal *
         ******************/

        // initialize goal with current position
        goal = pose;

        Pose frontier;
        double max_bid = 0;
        double bid;

        // get list of frontiers
        vector< vector <int> > frontiers = FrontiersReachable();

        // iterate through all frontiers
        vector< vector<int> >::iterator it;
        for(it=frontiers.begin(); it<frontiers.end(); ++it){
            // make pose of coordinates
            frontier.x = it->at(0);
            frontier.y = it->at(1);
            frontier.a = Angle(pose.x, pose.y, frontier.x, frontier.y);

            // calculate bid (negative of cost)
            bid = CalcBid(frontier);

            // invalid bid, not enough energy to reach frontier
            if(bid == BID_INV){
                continue;
            }

            // maximize bid, minimize cost
            if(bid > max_bid || max_bid == 0){
                max_bid = bid;
                goal = frontier;
            }
        }


        /********************************
         * coordinate with other robots *
         ********************************/

        // no new goal was found, coordinate docking with other robots
        if(goal == pose){
            // no reachable goal with full battery
            if(FullyCharged()){
                // try finding another docking station from where it is still possible to explore
                ds = cord->NextClosestDs(pos->GetPose(), RemainingDist());

                // start docking station auction
                if(ds.id > 0){
                    state = STATE_PRECHARGE;
                    cord->DockingAuction(pos->GetPose(), ds.id);
                }

                // end of exploration
                else
                    Finalize();
            }

            // needs recharging, coordinate with other robots
            else{
                state = STATE_PRECHARGE;

                // select docking station and store in private variable
                switch(OPT){
                    case OPT_ENERGY:
                        ds = cord->ClosestDs(pos->GetPose());
                        break;
                    case OPT_TIME:
                        ds = cord->ClosestFreeDs(pos->GetPose(), RemainingDist());
                        break;
                    case OPT_STABILITY:
                        printf("[%s:%d] [robot %d]: optimization of stability not yet implemented\n", StripPath(__FILE__), __LINE__, id);
                        break;
                    default:
                        printf("[%s:%d] [robot %d]: invalid optimization goal\n", StripPath(__FILE__), __LINE__, id);
                }

                // start docking station auction
                if(ds.id > 0){
                    cord->DockingAuction(pos->GetPose(), ds.id);
                }

                // no docking station found
                else
                    printf("[%s:%d] [robot %d]: no docking station found\n", StripPath(__FILE__), __LINE__, id);
            }
        }

        // goal found, coordinate exploration with other robots
        else{
            cord->FrontierAuction(goal, max_bid);
        }
    }

    void Robot::Move()
    {
        // don't move if charging, dead or done exploring
        if(state == STATE_CHARGE || state == STATE_DEAD || state == STATE_FINISHED)
            return;

        // robot already at current goal
        if(pos->GetPose().Distance(goal) < GOAL_TOLERANCE){
            // start recharging
            if(state == STATE_GOING_CHARGING){
                // stop moving
                pos->Stop();

                // charging state
                state = STATE_CHARGE;

                // subscribe to docking station
                ds.model->AddCallback(Model::CB_UPDATE, (model_callback_t)ChargingUpdate, this);
                ds.model->Subscribe();

                return;
            }

            if(state == STATE_CHARGE_QUEUE){
                // stop moving
                pos->Stop();
                return;
            }

            // no valid goal in the queue
            if(goal_next_bid == BID_INV){
                printf("[%s:%d] [robot %d]: invalid goal\n", StripPath(__FILE__), __LINE__, id);
                return;
            }

            // set goal and invalidate next goal
            goal = goal_next;
            goal_next_bid = BID_INV;
        }

        // store goal for visualization
        pos->waypoints.push_back(ModelPosition::Waypoint(goal, wpcolor));

        // move robot to frontier / docking station
        if(state == STATE_EXPLORE || state == STATE_GOING_CHARGING || state == STATE_CHARGE_QUEUE){
            pos->GoTo(goal);
        }
    }

    void Robot::Move(Pose to, double bid)
    {
        // robot is at goal, move to next goal
        if(to == goal || pos->GetPose().Distance(goal) < GOAL_TOLERANCE){
            goal = to;
            Move();
        }

        // store as next goal only
        // - if my bid is higher than for the other goal already stored
        // - if robot needs recharging (goal should be a docking station)
        else if(goal_next_bid == BID_INV || goal_next_bid < bid || state == STATE_GOING_CHARGING){
            goal_next = to;
            goal_next_bid = bid;
        }
    }

    double Robot::CalcBid(Pose frontier)
    {
        Pose pose = pos->GetPose();

        // find closest docking station and store in private variable
        ds = cord->ClosestDs(pose);

        // calculate distances
        double dg = pose.Distance(frontier);
        double dgb = Distance(frontier.x, frontier.y, ds.pose.x, ds.pose.y);

        // no reachable frontier
        if(RemainingDist() <= dg + dgb)
            return BID_INV;

        // calculate energy aware parameter
        double dgbe;
        if(pos->FindPowerPack()->ProportionRemaining() > CHARGE_TURN)
            dgbe = -dgb;
        else
            dgbe = dgb;

        // calculate angular parameter
        double theta = 1/PI * (PI - abs(abs(pose.a - Angle(pose.x, pose.y, frontier.x, frontier.y)) - PI));

        // calculate distance to other robots
        double dr = -cord->DistRobot(frontier);

        // calculate bid
        return -(W1*dg + W2*dgb + W3*dgbe + W4*theta + W5*dr);
    }

    OrthoCamera* Robot::GetCam()
    {
        return cam;
    }

    int Robot::GetId()
    {
        return id;
    }

    robot_state_t Robot::GetState()
    {
        return state;
    }

    void Robot::SetState(robot_state_t state)
    {
        this->state = state;
    }

    void Robot::SetPose(Pose pose)
    {
        pos->SetPose(pose);
    }

    Pose Robot::GetPose()
    {
        return pos->GetPose();
    }

    bool Robot::GoalQueue()
    {
        return goal_next_bid != BID_INV;
    }

    void Robot::Dock(ds_t ds, double bid)
    {
        // already charging, no docking required
        if(this->ds.id == ds.id && state == STATE_CHARGE)
            return;

        this->ds = ds;
        state = STATE_GOING_CHARGING;
        Move(ds.pose, bid);
    }

    void Robot::DockQueue(ds_t ds, double bid)
    {
        // already charging, no queueing required
        if(this->ds.id == ds.id && state == STATE_CHARGE)
            return;

        this->ds = ds;
        goal = ds.pose;
        state = STATE_CHARGE_QUEUE;
        Move(ds.pose, bid);
    }

    void Robot::UnDock()
    {
        // continue exploration
        state = STATE_EXPLORE;
        Explore();
    }

    double Robot::RemainingTime()
    {
        // calculate power consumption (according to stage model: libstage/model_position:496)
        double power = pos->velocity_bounds->max * WATTS_KGMS * pos->GetTotalMass() + WATTS;

        // calculate remaining distance
        return pos->FindPowerPack()->GetStored() / power;
    }

    double Robot::RemainingChargeTime()
    {
        return pos->FindPowerPack()->RemainingCapacity() / WATTS_CHARGE;
    }

    double Robot::MaxDist()
    {
        return RemainingDist(pos->FindPowerPack()->GetCapacity());
    }

    bool Robot::FullyCharged()
    {
        return pos->FindPowerPack()->ProportionRemaining() >= CHARGE_FULL;
    }

    bool Robot::Charging(ds_t& at)
    {
        if(state == STATE_CHARGE){
            at = ds;
            return true;
        }

        return false;
    }

    bool Robot::Docking(ds_t& at)
    {
        if(state == STATE_GOING_CHARGING || state == STATE_CHARGE){
            at = ds;
            return true;
        }

        return false;
    }

    bool Robot::Queueing(ds_t& at)
    {
        if(state == STATE_CHARGE_QUEUE){
            at = ds;
            return true;
        }

        return false;
    }

    GridMap* Robot::GetMap()
    {
        return map;
    }

    vector< vector <int> > Robot::FrontiersReachable()
    {
        return FrontiersReachable(pos->GetPose(), RemainingDist(), false);
    }

    vector< vector <int> > Robot::FrontiersReachable(Pose pos, double range, bool ds)
    {
        // get all frontiers
        vector< vector <int> > frontiers = map->Frontiers();

        // iterate through all frontiers
        vector< vector<int> >::iterator it;
        for(it=frontiers.end()-1; it>=frontiers.begin(); --it){
            // calculate distance
            double dist1 = Distance(pos.x, pos.y, it->at(0), it->at(1));
            double dist2;
            if(ds)
                dist2 = dist1;
            else
                dist2 = Distance(it->at(0), it->at(1), this->ds.pose.x, this->ds.pose.y);

            // remove frontier if it is not reachable
            if(range <= dist1 + dist2)
                frontiers.erase(it);
        }

        return frontiers;
    }

    void Robot::UpdateMap(GridMap* map)
    {
        // apply map updates
        this->map->Update(map);
    }

    void Robot::UpdateMap()
    {
        // get current position
        Pose pose = pos->GetPose();

        // mark local neighborhood as visited
        GridMap* local = map->Clear(pose);

        // visualize map progress
        map->VisualizeGui(pose);

        // share map with other robots in range
        cord->BroadcastMap(local);
    }

    double Robot::Distance(double from_x, double from_y, double to_x, double to_y)
    {
        double x = to_x - from_x;
        double y = to_y - from_y;
        return hypot(x,y);
    }

    double Robot::Distance(double to_x, double to_y)
    {
        return Distance(pos->GetPose().x, pos->GetPose().y, to_x, to_y);
    }

    double Robot::Angle(double from_x, double from_y, double to_x, double to_y)
    {
        double x = to_x - from_x;
        double y = to_y - from_y;
        return atan2(y,x);
    }

    double Robot::RemainingDist()
    {
        return RemainingDist(pos->FindPowerPack()->GetStored());
    }

    double Robot::RemainingDist(joules_t charge)
    {
        double velocity; // average velocity
        double power;    // power consumption at average velocity
        World* world = pos->GetWorld();

        // calculate average velocity
        if(dist_travel > 0 && world->SimTimeNow() > 0){
            velocity = dist_travel/world->SimTimeNow()*1000000;
        }
        // at beginning of simulation take max velocity
        else{
            velocity = pos->velocity_bounds->max / 2;
        }

        // calculate power consumption (according to stage model: libstage/model_position:496)
        power = velocity * WATTS_KGMS * pos->GetTotalMass() + WATTS;

        // calculate remaining distance
        return charge / power * velocity;
    }

    void Robot::Log()
    {
        log->Log(pos->GetWorld()->SimTimeNow(), dist_travel, map->Explored(), goal.x, goal.y, STATE_STRING[state]);
    }

    void Robot::Finalize()
    {
        state = STATE_FINISHED;
        printf("[%s:%d] [robot %d]: exploration finished\n", StripPath(__FILE__), __LINE__, id);

        // log data
        Log();

        // share map with other robots in range
        cord->BroadcastMap();

        // pause if all other robots finished already
        if(cord->Finished())
            pos->GetWorld()->Stop();
    }

    int Robot::PositionUpdate(ModelPosition* pos, Robot* robot)
    {
        // robot is done
        if(robot->state == STATE_FINISHED){
            return -1; // no more updates
        }

        // initialize robot
        if(robot->state == STATE_INIT){
            robot->Init();
        }

        // start exploration
        if(robot->state == STATE_IDLE){
            robot->Explore();
        }

        // clear map and compute traveled distance while traveling (not too often)
        Pose pose = pos->GetPose();
        double dist = pose.Distance(robot->last_pose);
        if(dist > MAP_UPDATE_DIST){
            // clear map
            robot->UpdateMap();

            // add distance to traveled distance
            robot->dist_travel += dist;

            // store last pose
            robot->last_pose = pose;
        }

        // no action required when charging
        if(robot->state == STATE_PRECHARGE || robot->state == STATE_CHARGE){
            return 0;
        }

        // robot reached goal
        if(pos->GetPose().Distance(robot->goal) < GOAL_TOLERANCE){
            // continue exploration
            if(robot->state == STATE_EXPLORE){
                // log data
                robot->Log();

                // continue exploration
                robot->Explore();
            }

            // robot needs recharging docking station
            else if(robot->state == STATE_GOING_CHARGING){
                if(robot->GoalQueue()){
                    if(robot->goal_next == robot->ds.pose){
                        robot->Move(robot->goal_next, robot->goal_next_bid);
                    }
                }
            }

            // robot waits for recharging at docking station
            else if(robot->state == STATE_CHARGE_QUEUE){
                    // start docking station auction
                    if(robot->ds.id > 0){
                        robot->cord->DockingAuction(pos->GetPose(), robot->ds.id);
                    }

                    // no docking station found
                    else
                        printf("[%s:%d] [robot %d]: no docking station found\n", StripPath(__FILE__), __LINE__, robot->id);
            }

            else{
                printf("[%s:%d] [robot %d]: invalid state: %d\n", StripPath(__FILE__), __LINE__, robot->id, robot->state);
            }
        }

        return 0; // run again
    }

    int Robot::FiducialUpdate(ModelFiducial* fid, Robot* robot)
    {
        // get all fiducials
        std::vector<ModelFiducial::Fiducial>& fids = fid->GetFiducials();
        std::vector<ModelFiducial::Fiducial>::iterator it;

        // robots is on its way for recharging
        if(robot->state == STATE_GOING_CHARGING){
            // check fiducial return signal
            for(it = fids.begin(); it<fids.end(); ++it){
                // fiducial is my docking station
                if(it->id == robot->ds.id){
                    // move robot to docking station
                    robot->goal.x = it->pose.x;
                    robot->goal.y = it->pose.y;
                    robot->goal.a = it->bearing;
                    robot->Move();
                }
            }
        }

        // robot is currently recharging
        else if(robot->state == STATE_CHARGE){
            // robot is done charging
            if(robot->FullyCharged()){
                // log data
                robot->Log();

                // docking station is free now
                robot->cord->DsVacant(robot->ds.id);

                // invalidate any previous goals
                robot->goal_next_bid = BID_INV;

                // continue exploration
                robot->state = STATE_EXPLORE;
                robot->Explore();
            }
        }

        // store docking stations
        else{
            for(it = fids.begin(); it<fids.end(); ++it){
                robot->cord->UpdateDs(it->id, it->pose);
            }
        }

        return 0; // run again
    }

    int Robot::ChargingUpdate(Model* mod, Robot* robot)
    {
        // stop recharging
        if(robot->FullyCharged() || robot->state != STATE_CHARGE){
            robot->UnDock();
            return -1; // no more updates
        }

        // start recharging
        if(robot->pos->FindPowerPack()->GetCharging() == false){
            robot->pos->FindPowerPack()->ChargeStart();
        }

        // charge with a constant rate
        if(robot->pos->GetWorld()->SimTimeNow() < robot->last_charge + CHARGE_RATE){
            return 0;
        }
        robot->last_charge = robot->pos->GetWorld()->SimTimeNow();

        // look for power output of docking station in world file
        if(robot->charging_watt <= 0){
            for(int i=0; i<robot->pos->GetWorld()->GetWorldFile()->GetEntityCount(); ++i){
                robot->charging_watt = robot->pos->GetWorld()->GetWorldFile()->ReadFloat(i, "give_watts", robot->charging_watt);
                if(robot->charging_watt > 0)
                    break;
            }
        }

        // add watts to robot's power pack according to rate
        robot->pos->FindPowerPack()->Add(robot->charging_watt);

        return 0; // run again
    }
}
