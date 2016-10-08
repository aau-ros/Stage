#include "eae_robot.hh"

using namespace Stg;
using namespace std;

namespace eae
{
    Robot::Robot(ModelPosition* pos)
    {
        // robot id
        id = pos->GetId();

        // instantiate objects
        this->pos = pos;
        fid = (ModelFiducial*)pos->GetUnusedModelOfType("fiducial");
        laser = (ModelRanger*)pos->GetChild("ranger:0");
        map = new GridMap(pos, id);
        cord = new Coordination(pos, this);
        log = new LogOutput(pos, id, cord->GetWifiModel(), cord->GetStrategy(), cord->GetStrategyString(), cord->GetPolicy(), cord->GetPolicyString(), pos->FindPowerPack()->GetCapacity(), MapName());
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

        // subscribe to laser ranger to receive data
        laser->Subscribe();

        // store starting point for visualization
        pos->waypoints.push_back(ModelPosition::Waypoint(pos->GetPose(), wpcolor));

        // distance traveled
        dist_travel = 0;

        // goal queue is empty
        goal_next_bid = BID_INV;

        // charging power, look for power output of docking station in world file
        charging_watt = 0;
        for(int i=0; i<pos->GetWorld()->GetWorldFile()->GetEntityCount(); ++i){
            charging_watt = pos->GetWorld()->GetWorldFile()->ReadFloat(i, "give_watts", charging_watt);
            if(charging_watt > 0)
                break;
        }

        // last charging time
        last_charge = 0;

        // no path planned yet
        valid_path = false;

        // waiting time at docking stations
        waiting_time = 0;
        waiting_start = 0;

        // robot did not try turning to fix computing path
        turning = 0;
        turned = 0;

        // no ds selected yet
        ds = NULL;
        ds_prev = NULL;
    }

    Robot::~Robot()
    {
        delete map;
        delete cord;
        delete log;
        delete cam;
    }

    void Robot::Init()
    {
        if(InArray(id, DEBUG_ROBOTS, sizeof(DEBUG_ROBOTS)/sizeof(id)))
            printf("[%s:%d] [robot %d]: init\n", StripPath(__FILE__), __LINE__, id);

        // do an initial map update
        UpdateMap();

        // pose at last map update
        last_pose = pos->GetPose();

        // try to select docking station
        ds = cord->SelectDs(RemainingDist());

        // turn to find docking station around
        if(!ds){
            pos->SetTurnSpeed(pos->velocity_bounds[3].max);

            // return, init will be called again
            return;
        }

        // stop turning
        pos->SetTurnSpeed(0);

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

        // only explore if currently not traveling on a path
        if(valid_path)
            return;

        // get current position
        Pose pose = pos->GetPose();

        if(InArray(id, DEBUG_ROBOTS, sizeof(DEBUG_ROBOTS)/sizeof(id)))
            printf("[%s:%d] [robot %d]: exploring\n", StripPath(__FILE__), __LINE__, id);

        // visualize map progress
//         map->VisualizeGui(pos->GetPose());


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

        if(InArray(id, DEBUG_ROBOTS, sizeof(DEBUG_ROBOTS)/sizeof(id)))
            printf("[%s:%d] [robot %d]: frontiers reachable %lu\n", StripPath(__FILE__), __LINE__, id, frontiers.size());

        // iterate through all frontiers
        vector< vector<int> >::iterator it;
        for(it=frontiers.begin(); it<frontiers.end(); ++it){
            if(SamePoint(pose, Pose(it->at(0), it->at(1), 0, 0)) || SamePoint(goal_prev, Pose(it->at(0), it->at(1), 0, 0)))
                continue;

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
        if(SamePoint(goal, pose) || SamePoint(goal, goal_prev)){
            if(InArray(id, DEBUG_ROBOTS, sizeof(DEBUG_ROBOTS)/sizeof(id)))
                printf("[%s:%d] [robot %d]: no reachable frontiers\n", StripPath(__FILE__), __LINE__, id);

            // no reachable goal with full battery
            if(FullyCharged()){
                if(InArray(id, DEBUG_ROBOTS, sizeof(DEBUG_ROBOTS)/sizeof(id)))
                    printf("[%s:%d] [robot %d]: look for another ds\n", StripPath(__FILE__), __LINE__, id);

                // try finding another docking station from where it is still possible to explore
                Ds* ds_op = cord->SelectDs(RemainingDist(), POL_OPPORTUNE, ds->id, true);

                if(InArray(id, DEBUG_ROBOTS, sizeof(DEBUG_ROBOTS)/sizeof(id)))
                    printf("[%s:%d] [robot %d]: try ds %d\n", StripPath(__FILE__), __LINE__, id, ds_op->id);

                // start docking station auction
                if(ds_op && SamePoint(ds->pose, ds_op->pose) == false){
                    ds = ds_op;

                    state = STATE_PRECHARGE;

                    // start recharging procedure
                    cord->Recharge();
                }

                // end of exploration
                else{
                    Finalize();
                }
            }

            // needs recharging, coordinate with other robots
            else{
                if(InArray(id, DEBUG_ROBOTS, sizeof(DEBUG_ROBOTS)/sizeof(id)))
                    printf("[%s:%d] [robot %d]: needs recharging\n", StripPath(__FILE__), __LINE__, id);

                state = STATE_PRECHARGE;

                // have valid docking station
                if(ds > 0){
                    // start recharging procedure
                    cord->Recharge();
                }

                // no docking station found
                else{
                    printf("[%s:%d] [robot %d]: no docking station found\n", StripPath(__FILE__), __LINE__, id);
                    Explore();
                }
            }
        }

        // goal found, coordinate exploration with other robots
        else{
            cord->FrontierAuction(goal, max_bid);
        }
    }

    void Robot::Move(bool clear)
    {
        // don't move if charging, dead or done exploring
        if(state == STATE_CHARGE || state == STATE_DEAD || state == STATE_FINISHED)
            return;

        // move robot to frontier / docking station
        if(state == STATE_EXPLORE || state == STATE_GOING_CHARGING || state == STATE_CHARGE_QUEUE){
            // current pose
            Pose pose = pos->GetPose();

            // make a new plan
            if(clear){
                if(InArray(id, DEBUG_ROBOTS, sizeof(DEBUG_ROBOTS)/sizeof(id)))
                    printf("[%s:%d] [robot %d]: moving on new path to (%.2f,%.2f)\n", StripPath(__FILE__), __LINE__, id, goal.x, goal.y);

                // plan path to goal
                valid_path = Plan(pose, goal);

                // no plan found, choose next goal
                if(!valid_path){
                    // try if turning fixes the problem
                    // try at least a couple of times
                    if(turning < TURN_TRIALS){
                        ++turning;
                        pos->GoTo(pose.x, pose.y, pose.a+PI);
                        return;
                    }

                    // it didn't work
                    else{
                        printf("[%s:%d] [robot %d]: failed to find path from (%.2f,%.2f) to (%.2f,%.2f)\n", StripPath(__FILE__), __LINE__, id, pose.x, pose.y, goal.x, goal.y);

                        // stop moving
                        pos->Stop();

                        // go to next goal if there is one
                        if(GoalQueue())
                            SetGoalNext();

                        // find a new goal if the robot is exploring
                        else if(state == STATE_EXPLORE)
                            Explore();

                        // it just didn't work, go on a straight line towards goal
                        else
                            pos->GoTo(goal);
                    }

                    return;
                }

                // try turning again next time a path cannot be computed
                turning = 0;
                turned = 0;

                // store goal for visualization
                pos->waypoints.push_back(ModelPosition::Waypoint(goal, wpcolor));

                // visualize path TODO: not working!
//                 if(valid_path){
//                     GraphVis* vis_path = new GraphVis(&path);
//                     vis_path->Visualize(pos, cam);
//                     this->path->Draw();
//                 }
            }

            // robot has a path, keep following
            if(valid_path){
                double a_goal, a_error;

                // get angular velocity to reach next node on path
                path->GoodDirection(pose, 2, a_goal);
                a_error = normalize(a_goal - pose.a);

                // set velocities and avoid obstacles
                SetMotorSpeed(a_error);
            }
        }
    }

    void Robot::SetGoal(Pose to, double bid)
    {
        // robot is at current goal
        // or does not have a goal
        // use euclidean distance
        if(pos->GetPose().Distance(goal) < GOAL_TOLERANCE || ((state == STATE_EXPLORE || state == STATE_GOING_CHARGING || state == STATE_CHARGE_QUEUE) && valid_path == false)){
            if(InArray(id, DEBUG_ROBOTS, sizeof(DEBUG_ROBOTS)/sizeof(id)))
                printf("[%s:%d] [robot %d]: set goal (%.2f,%.2f)\n", StripPath(__FILE__), __LINE__, id, to.x, to.y);

            // store previous goal
            goal_prev = goal;

            // goal in queue
            if(GoalQueue()){
                // go to goal in queue
                goal = goal_next;
                goal_next_bid = BID_INV;

                // enqueue new goal
                goal_next = to;
                goal_next_bid = bid;
            }

            // no goal in queue, go to new goal
            else{
                goal = to;
            }

            // move towards goal
            Move(true);
        }

        // store as next goal only
        // - if my bid is higher than for the other goal already stored
        // - if robot needs recharging (goal should be a docking station)
        else if(GoalQueue() == false || goal_next_bid < bid || state == STATE_GOING_CHARGING || state == STATE_CHARGE_QUEUE){
            if(InArray(id, DEBUG_ROBOTS, sizeof(DEBUG_ROBOTS)/sizeof(id)))
                printf("[%s:%d] [robot %d]: enqueue goal (%.2f,%.2f)\n", StripPath(__FILE__), __LINE__, id, to.x, to.y);
            goal_next = to;
            goal_next_bid = bid;
        }
        else{
            if(InArray(id, DEBUG_ROBOTS, sizeof(DEBUG_ROBOTS)/sizeof(id)))
                printf("[%s:%d] [robot %d]: discard goal (%.2f,%.2f)\n", StripPath(__FILE__), __LINE__, id, to.x, to.y);
        }
    }

    void Robot::SetGoalNext()
    {
        if(InArray(id, DEBUG_ROBOTS, sizeof(DEBUG_ROBOTS)/sizeof(id)))
            printf("[%s:%d] [robot %d]: set goal next (%.2f,%.2f)\n", StripPath(__FILE__), __LINE__, id, goal_next.x, goal_next.y);

        // store previous goal
        goal_prev = goal;

        // set new goal
        goal = goal_next;

        // invalidate next goal
        goal_next_bid = BID_INV;

        // start moving there
        Move(true);
    }

    double Robot::CalcBid(Pose frontier)
    {
        if(!ds)
            return BID_INV;

        Pose pose = pos->GetPose();

        // calculate distances
        int dg = Distance(frontier.x, frontier.y);
        if(dg < 0)
            return BID_INV;
        int dgb = map->Distance(frontier.x, frontier.y, ds->pose.x, ds->pose.y);
        if(dgb < 0)
            return BID_INV;

        // no reachable frontier
        if(RemainingDist() <= dg + dgb)
            return BID_INV;

        // calculate energy aware parameter
        double dgbe = 0;
        if(W3 != 0){
            if(pos->FindPowerPack()->ProportionRemaining() > CHARGE_TURN)
                dgbe = -dgb;
            else
                dgbe = dgb;
        }

        // calculate angular parameter
        double theta = 0;
        if(W4 != 0)
            theta = 1/PI * (PI - abs(abs(pose.a - Angle(pose.x, pose.y, frontier.x, frontier.y)) - PI));

        // calculate distance to other robot's goals
        double dr = 0;
        if(W5 != 0)
            dr = -cord->DistRobot(frontier);

        // calculate bid
        return -(W1*dg + W2*dgb + W3*dgbe + W4*theta + W5*dr);
    }

    bool Robot::Plan(Pose start_pose, Pose goal_pose)
    {
        // execute a* algorithm
        vector<ast::point_t> path;
        if(map->AStar(start_pose, goal_pose, &path) == false)
            return false;

        // remove old path
        if(valid_path){
            delete this->path;
            valid_path = false;
        }

        // generate path as graph
        this->path = new Graph(id);
        unsigned int dist = 0;
        Node* last_node = NULL;
        vector<ast::point_t>::reverse_iterator rit;
        for(rit = path.rbegin(); rit != path.rend(); ++rit){
            grid_cell_t c;
            c.x = rit->x;
            c.y = rit->y;
            Node* node = new Node(map->C2M(c), dist++);

            this->path->AddNode(node);

            if(last_node)
                last_node->AddEdge(new Edge(node));

            last_node = node;
        }

        return true;
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

    Pose Robot::GetPose()
    {
        return pos->GetPose();
    }

    Ds* Robot::GetDs()
    {
        return ds;
    }

    bool Robot::GoalQueue()
    {
        return goal_next_bid != BID_INV;
    }

    void Robot::Dock(Ds* ds, double bid)
    {
        // invalid docking station
        if(!ds)
            return;

        if(InArray(id, DEBUG_ROBOTS, sizeof(DEBUG_ROBOTS)/sizeof(id)))
            printf("[%s:%d] [robot %d]: docking\n", StripPath(__FILE__), __LINE__, id);

        // already charging, no docking required
        if(this->ds->id == ds->id && state == STATE_CHARGE)
            return;

        // measure waiting time
        if(state == STATE_CHARGE_QUEUE && waiting_start > 0){
            waiting_time += pos->GetWorld()->SimTimeNow() / 1000000 - waiting_start;
            waiting_start = 0;
        }

        this->ds = ds;
        state = STATE_GOING_CHARGING;
        SetGoal(ds->pose, bid);
    }

    void Robot::DockQueue(Ds* ds, double bid)
    {
        // invalid docking station
        if(!ds)
            return;

        if(InArray(id, DEBUG_ROBOTS, sizeof(DEBUG_ROBOTS)/sizeof(id)))
            printf("[%s:%d] [robot %d]: docking queue\n", StripPath(__FILE__), __LINE__, id);

        // already charging, no queueing required
        if(this->ds->id == ds->id && state == STATE_CHARGE)
            return;

        this->ds = ds;
        state = STATE_CHARGE_QUEUE;
        SetGoal(ds->pose, bid);
        waiting_start = pos->GetWorld()->SimTimeNow() / 1000000;
    }

    void Robot::UnDock()
    {
        if(InArray(id, DEBUG_ROBOTS, sizeof(DEBUG_ROBOTS)/sizeof(id)))
            printf("[%s:%d] [robot %d]: undock\n", StripPath(__FILE__), __LINE__, id);

        // stop moving
        pos->Stop();

        // remove current path plan
        if(valid_path){
            delete path;
            valid_path = false;
        }

        // log data
        Log();

        // docking station is free now
        cord->DsVacant(ds->id);

        // invalidate any previous goals
        goal_next_bid = BID_INV;

        // continue exploration
        state = STATE_EXPLORE;
        Explore();
    }

    double Robot::RemainingTime()
    {
        // calculate remaining distance
        return pos->FindPowerPack()->GetStored() / POWER_M;
    }

    double Robot::RemainingChargeTime()
    {
        return pos->FindPowerPack()->RemainingCapacity() / charging_watt;
    }

    double Robot::MaxDist()
    {
        return RemainingDist(pos->FindPowerPack()->GetCapacity());
    }

    bool Robot::FullyCharged()
    {
        return pos->FindPowerPack()->ProportionRemaining() >= CHARGE_FULL;
    }

    Ds* Robot::Charging()
    {
        if(state == STATE_CHARGE){
            return ds;
        }

        return NULL;
    }

    Ds* Robot::Docking()
    {
        if(state == STATE_GOING_CHARGING || state == STATE_CHARGE){
            return ds;
        }

        return NULL;
    }

    Ds* Robot::Queueing()
    {
        if(state == STATE_CHARGE_QUEUE){
            return ds;
        }

        return NULL;
    }

    Ds* Robot::QueueingAtDs()
    {
        if(state == STATE_CHARGE_QUEUE && ds && pos->GetPose().Distance(ds->pose) < GOAL_TOLERANCE){
            return ds;
        }

        return NULL;
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
            // remove frontier if it has been auctioned already
            if(cord->OldFrontier(Pose(it->at(0), it->at(1), 0, 0))){
                frontiers.erase(it);
                continue;
            }

            // calculate distance
            double dist1 = map->Distance(pos.x, pos.y, it->at(0), it->at(1));
            double dist2;
            if(ds)
                dist2 = dist1;
            else
                dist2 = map->Distance(it->at(0), it->at(1), this->ds->pose.x, this->ds->pose.y);

            // remove frontier if it is not reachable
            if(range <= dist1 + dist2){
                frontiers.erase(it);
            }
        }

        return frontiers;
    }

    int Robot::Distance(double to_x, double to_y)
    {
        return map->Distance(pos->GetPose().x, pos->GetPose().y, to_x, to_y);
    }

    bool Robot::SamePoint(Pose point1, Pose point2)
    {
        return point1.Distance(point2) <= EPSILON; // euclidean
    }

    void Robot::Continue()
    {
        printf("[%s:%d] [robot %d]: continue exploration\n", StripPath(__FILE__), __LINE__, id);

        // reset state
        state = STATE_IDLE;

        // subscribe to position update again
        pos->AddCallback(Model::CB_UPDATE, (model_callback_t)PositionUpdate, this);
        pos->Subscribe();

        // continue exploration
        Explore();
    }

    void Robot::UpdateMap(GridMap* map)
    {
        // apply map updates
        this->map->Update(map);

        // visualize map progress
        //this->map->VisualizeGui(pos->GetPose());
    }

    void Robot::UpdateMap()
    {
        // get current position
        Pose pose = pos->GetPose();

        // mark local neighborhood as visited
        GridMap* local = map->Clear(pose, laser->GetSensors()[0].ranges);

        // visualize map progress
        //map->VisualizeGui(pose);
        //map->Visualize(pose);

        // share map with other robots in range
        cord->BroadcastMap(local);
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
        // calculate remaining distance
        return charge / POWER_M * pos->velocity_bounds->max;
    }

    void Robot::Log()
    {
        log->Log(pos->GetWorld()->SimTimeNow(), dist_travel, map->Explored(), goal.x, goal.y, STATE_STRING[state], waiting_time, ds->id, cord->msgs_sent, cord->msgs_received, cord->bytes_sent, cord->bytes_received);
    }

    void Robot::Finalize()
    {
        state = STATE_FINISHED;
        printf("[%s:%d] [robot %d]: exploration finished\n", StripPath(__FILE__), __LINE__, id);

        // stop moving
        pos->Stop();

        // log data
        Log();

        // share map with other robots in range
        cord->BroadcastMap();

        // pause if all other robots finished already
        if(cord->Finished())
            pos->GetWorld()->Stop();
    }

    void Robot::SetMotorSpeed(double direction)
    {
        // speed of the robot
        double turn_speed = direction;
        double x_speed;

        // calculate forward speed according to turning speed
        // the more the robot turns, the slower it goes forward
        if(abs(turn_speed) < 0.5)
            x_speed = pos->velocity_bounds->max;
        else if(0.5 <= abs(turn_speed) && abs(turn_speed) < 1.5)
            x_speed = pos->velocity_bounds->max * (1.5 - abs(turn_speed));
        else
            x_speed = 0;

        // set speed
        pos->SetXSpeed(x_speed);
        pos->SetTurnSpeed(turn_speed);
    }

    string Robot::MapName()
    {
        string name = "";
        for(int i=0; i<pos->GetWorld()->GetWorldFile()->GetEntityCount(); ++i){
            name = pos->GetWorld()->GetWorldFile()->ReadString(i, "bitmap", name);
            if(name != "")
                break;
        }
        return name;
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
            return 0;
        }

        // start exploration
        if(robot->state == STATE_IDLE){
            robot->Explore();
            return 0;
        }

        // clear map and compute traveled distance while traveling (not too often)
        Pose pose = pos->GetPose();
        double dist = pose.Distance(robot->last_pose); // euclidean
        bool map_update = false;
        if(dist > MAP_UPDATE_DIST || abs(pose.a-robot->last_pose.a) > MAP_UPDATE_ANGLE){
            // clear map
            robot->UpdateMap();

            // add distance to traveled distance
            robot->dist_travel += dist;

            // store last pose
            robot->last_pose = pose;

            // remember that the map was updated
            map_update = true;

            // measure angle the robot has turned to fix path computation
            if(robot->turning > 0){
                robot->turned += MAP_UPDATE_ANGLE;

                // try again to compute path
                if(robot->turned > robot->turning*PI/2){
                    robot->Move(true);
                    return 0;
                }
            }
        }

        // no action required when charging
        if(robot->state == STATE_PRECHARGE || robot->state == STATE_CHARGE){
            return 0;
        }

        // robot reached goal
        if(pos->GetPose().Distance(robot->goal) < GOAL_TOLERANCE){ // euclidean
            // remove current path plan
            if(robot->valid_path){
                delete robot->path;
                robot->valid_path = false;
            }

            // log data
            robot->Log();

            // robot needs recharging
            if(robot->state == STATE_GOING_CHARGING){
                // already at docking station
                if(robot->SamePoint(robot->goal, robot->ds->pose)){
                    // stop moving
                    pos->Stop();

                    // charging state
                    robot->state = STATE_CHARGE;

                    // subscribe to docking station
                    robot->ds->model->AddCallback(Model::CB_UPDATE, (model_callback_t)ChargingUpdate, robot);
                    robot->ds->model->Subscribe();
                }

                // next goal is docking station
                else if(robot->GoalQueue() && robot->SamePoint(robot->goal_next, robot->ds->pose)){
                    robot->SetGoalNext();
                }
            }

            // robot waits for recharging at docking station
            else if(robot->state == STATE_CHARGE_QUEUE){
                // already at docking station
                if(robot->SamePoint(robot->goal, robot->ds->pose)){
                    // stop moving
                    pos->Stop();

                    // coordinate docking station access
                    robot->cord->Recharge();
                }

                // next goal is docking station
                else if(robot->GoalQueue() && robot->SamePoint(robot->goal_next, robot->ds->pose)){
                    robot->SetGoalNext();
                }
            }

            // go to next goal
            else if(robot->GoalQueue()){
                robot->SetGoalNext();
            }

            // continue exploration
            else if(robot->state == STATE_EXPLORE){
                // clear map, in case the two goals where closer than MAP_UPDATE_DIST
                if(!map_update)
                    robot->UpdateMap();

                robot->Explore();
            }

            else{
                printf("[%s:%d] [robot %d]: invalid state: %s\n", StripPath(__FILE__), __LINE__, robot->id, STATE_STRING[robot->state].c_str());
            }

            return 0;
        }

        // robot is following the path to the next step, continue moving
        if(robot->valid_path){
            robot->Move();
            return 0;
        }

        // stop moving if not currently turning to aid path planning
        else if(robot->turning <= 0){
            robot->pos->Stop();
        }

        return 0; // run again
    }

    int Robot::FiducialUpdate(ModelFiducial* fid, Robot* robot)
    {
        // get all fiducials
        vector<ModelFiducial::Fiducial>& fids = fid->GetFiducials();
        vector<ModelFiducial::Fiducial>::iterator it;

        // store docking stations
        for(it = fids.begin(); it<fids.end(); ++it){
            robot->cord->UpdateDs(it->id, it->pose);
        }

        return 0; // run again
    }

    int Robot::ChargingUpdate(Model* mod, Robot* robot)
    {
        // stop recharging
        if(robot->pos->FindPowerPack()->ProportionRemaining() >= CHARGE_UNTIL || robot->state != STATE_CHARGE){
            robot->pos->FindPowerPack()->ChargeStop();
            robot->UnDock();
            return -1; // no more updates
        }

        // time since last charging cycle
        usec_t time = robot->pos->GetWorld()->SimTimeNow() - robot->last_charge;
        robot->last_charge = robot->pos->GetWorld()->SimTimeNow();

        // start recharging
        if(robot->pos->FindPowerPack()->GetCharging() == false){
            robot->pos->FindPowerPack()->ChargeStart();
            return 0;
        }

        // calculate joules for this charging cylce
        joules_t joules = robot->charging_watt * (double)time / 1000000.0;

        // add joules to robot's power pack
        robot->pos->FindPowerPack()->Add(joules);

        return 0; // run again
    }
}
