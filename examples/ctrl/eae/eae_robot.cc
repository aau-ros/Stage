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
//                     printf("[%s:%d] [robot %d]: starting auction\n", StripPath(__FILE__), __LINE__, id);
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
                        printf("[%s:%d] [robot %d]: invalid optimization goal %d\n", StripPath(__FILE__), __LINE__, id, OPT);
                }

                // start docking station auction
                if(ds.id > 0)
                    cord->DockingAuction(pos->GetPose(), ds.id);

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
            // check if there is a valid goal in the queue
            if(goal_next_bid != BID_INV){
                // set goal and invalidate next goal
                goal = goal_next;
                goal_next_bid = BID_INV;
            }

            // no valid goal
            else{
                printf("[%s:%d] [robot %d]: invalid goal\n", StripPath(__FILE__), __LINE__, id);
                return;
            }
        }

        // store goal for visualization
        pos->waypoints.push_back(ModelPosition::Waypoint(goal, wpcolor));

        // store traveled distance
        dist_travel += pos->GetPose().Distance(goal);

        // move robot to frontier
        if(state == STATE_EXPLORE){
            pos->GoTo(goal);
        }

        // move robot to docking station
        else if(state == STATE_PRECHARGE){
            // stop moving once the recharging starts
            if(pos->FindPowerPack()->GetCharging()){
                pos->Stop();
                state = STATE_CHARGE;
            }

            // move to docking station
            else{
                pos->GoTo(goal);
            }
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
        else if(goal_next_bid == BID_INV || goal_next_bid < bid || state == STATE_PRECHARGE){
            goal_next = to;
            goal_next_bid = bid;
        }

//         else{
//             printf("[%s:%d] [robot %d]: cannot move to (%.0f,%.0f), already moving to (%0.f,%.0f) then (%.0f,%.0f)!\n", StripPath(__FILE__), __LINE__, id, to.x, to.y, goal.x, goal.y, goal_next.x, goal_next.y);
//         }
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
        this->ds = ds;
        state = STATE_PRECHARGE;
        Move(ds.pose, bid);
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

    bool Robot::Docking(ds_t& at)
    {
        if(state == STATE_PRECHARGE || state == STATE_CHARGE){
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
        this->map->Update(map, pos->GetPose());

        // visualize map progress
        map->VisualizeGui(pos->GetPose());
    }

    void Robot::UpdateMap()
    {
        // get current position
        Pose pose = pos->GetPose();

        // mark neighborhood as visited
        map->Clear(round(pose.x), round(pose.y));

        // visualize map progress
        map->VisualizeGui(pose);

        // share map with other robots in range
        cord->BroadcastMap();
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
            velocity = pos->velocity_bounds->max;
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

        // clear map while traveling (not too often)
        Pose pose = pos->GetPose();
        if(pose.Distance(robot->last_pose) > MAP_UPDATE_DIST){
            robot->UpdateMap();
            robot->last_pose = pose;
        }

        // robot reached goal
        if(pos->GetPose().Distance(robot->goal) < GOAL_TOLERANCE){
            // continue exploration
            if(robot->state == STATE_EXPLORE){
                // log data
                robot->Log();

                // there is still a goal in the queue
//                 if(robot->GoalQueue()){
//                     robot->Move();
//                 }
//
//                 // otherwise just find a new goal
//                 else{
                    robot->Explore();
//                 }
            }

            // robot needs recharging
            else if(robot->state == STATE_PRECHARGE){
                // robot already at docking station, continue docking
                // or robot just finished exploring and will now go to docking station
//                 if(robot->goal == robot->ds.pose || robot->GoalQueue()){
//                     robot->Move();
//                 }
//
//                 // robot is waiting for docking station, start new auction
//                 else{
                    // select docking station and store in private variable
                    switch(OPT){
                        case OPT_ENERGY:
                            robot->ds = robot->cord->ClosestDs(pos->GetPose());
                            break;
                        case OPT_TIME:
                            robot->ds = robot->cord->ClosestFreeDs(pos->GetPose(), robot->RemainingDist());
                            break;
                        case OPT_STABILITY:
                            printf("[%s:%d] [robot %d]: optimization of stability not yet implemented\n", StripPath(__FILE__), __LINE__, robot->id);
                            break;
                        default:
                            printf("[%s:%d] [robot %d]: invalid optimization goal %d\n", StripPath(__FILE__), __LINE__, robot->id, OPT);
                    }

                    // start docking station auction
                    if(robot->ds.id > 0)
                        robot->cord->DockingAuction(pos->GetPose(), robot->ds.id);

                    // no docking station found
                    else
                        printf("[%s:%d] [robot %d]: no docking station found\n", StripPath(__FILE__), __LINE__, robot->id);
//                 }
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
        if(robot->state == STATE_PRECHARGE){
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
}
