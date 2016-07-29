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
        laser = (ModelRanger*)pos->GetChild( "ranger:1" );
        sonar = (ModelRanger*)pos->GetChild( "ranger:0" );
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

        // subscribe to laser ranger to receive data
        laser->Subscribe();

        // subscribe to sonar ranger to receive data
        sonar->Subscribe();

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

        // TODO
        avoidcount = 0;
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

        // only explore if the robot is not avoiding an obstacle
//         if(ObstacleAvoid())
//             return;

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
            if(it->at(0) == pose.x && it->at(1) == pose.y)
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

    void Robot::Move(bool clear)
    {
        // don't move if charging, dead or done exploring
        if(state == STATE_CHARGE || state == STATE_DEAD || state == STATE_FINISHED)
            return;

        // don't move if the robot is avoiding an obstacle
//         if(ObstacleAvoid())
//             return;

        // move robot to frontier / docking station
        if(state == STATE_EXPLORE || state == STATE_GOING_CHARGING || state == STATE_CHARGE_QUEUE){
            // make a new plan
            if(clear){
                // plan path to goal
                valid_path = Plan(pos->GetPose(), goal);

                // store goal for visualization
                pos->waypoints.push_back(ModelPosition::Waypoint(goal, wpcolor));
            }

            // robot has a path, keep following
            if(valid_path){
                double a_goal, a_error;

                // get angular velocity
                path->GoodDirection(pos->GetPose(), 5.0, a_goal);
                a_error = normalize(a_goal - pos->GetPose().a);

                // set velocities
                pos->SetTurnSpeed(a_error);
                pos->SetXSpeed(pos->velocity_bounds->max);
            }
        }
    }

    void Robot::SetGoal(Pose to, double bid)
    {
        // robot is at goal
        // or does not have a goal
        if(pos->GetPose().Distance(goal) < GOAL_TOLERANCE || ((state == STATE_EXPLORE || state == STATE_GOING_CHARGING) && valid_path == false)){
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
        else if(GoalQueue() == false || goal_next_bid < bid || state == STATE_GOING_CHARGING || STATE_CHARGE_QUEUE){
            goal_next = to;
            goal_next_bid = bid;
        }
    }

    void Robot::SetGoalNext()
    {
        // set new goal
        goal = goal_next;

        // invalidate next goal
        goal_next_bid = BID_INV;

        // start moving there
        Move(true);
    }

    double Robot::CalcBid(Pose frontier)
    {
        Pose pose = pos->GetPose();

        // find closest docking station and store in private variable
        ds = cord->ClosestDs(pose);

        // calculate distances
        int dg = Distance(frontier.x, frontier.y);
        if(dg < 0)
            return BID_INV;
        int dgb = Distance(frontier.x, frontier.y, ds.pose.x, ds.pose.y);
        if(dgb < 0)
            return BID_INV;

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

    bool Robot::Plan(Pose start_pose, Pose goal_pose)
    {
        // execute a* algorithm
        vector<ast::point_t> path;
        if(AStar(start_pose, goal_pose, &path) == false)
            return false;

        // remove old path
        if(valid_path){
            delete this->path;
            valid_path = false;
        }

        // generate path as graph
        this->path = new Graph();
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

    bool Robot::AStar(Pose start_pose, Pose goal_pose, vector<ast::point_t>* path)
    {
        // start and goal
        grid_cell_t start_cell = map->M2C(start_pose);
        grid_cell_t goal_cell = map->M2C(goal_pose);
        ast::point_t start(start_cell.x, start_cell.y);
        ast::point_t goal(goal_cell.x, goal_cell.y);

        // find path
        bool result = ast::astar(map->Rasterize(), (uint32_t)map->Width(), (uint32_t)map->Height(), start, goal, *path);

        // error
        if(!result){
            printf("[%s:%d] [robot %d]: failed to find path from (%.2f,%.2f) to (%.2f,%.2f)\n", StripPath(__FILE__), __LINE__, id, start_pose.x, start_pose.y, goal_pose.x, goal_pose.y);
            return false;
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
        SetGoal(ds.pose, bid);
    }

    void Robot::DockQueue(ds_t ds, double bid)
    {
        // already charging, no queueing required
        if(this->ds.id == ds.id && state == STATE_CHARGE)
            return;

        this->ds = ds;
        state = STATE_CHARGE_QUEUE;
        SetGoal(ds.pose, bid);
    }

    void Robot::UnDock()
    {
        // log data
        Log();

        // docking station is free now
        cord->DsVacant(ds.id);

        // invalidate any previous goals
        goal_next_bid = BID_INV;

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

        // visualize map progress
        map->VisualizeGui(pos->GetPose());
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

    int Robot::Distance(double from_x, double from_y, double to_x, double to_y)
    {
        // execute a* algorithm
        vector<ast::point_t> path;
        if(AStar(Pose(from_x, from_y, 0, 0), Pose(to_x, to_y, 0, 0), &path) == false)
            return -1;

        return (int)path.size();
    }

    int Robot::Distance(double to_x, double to_y)
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

    bool Robot::ObstacleAvoid()
    {
        bool obstruction = false;
        bool stop = false;

        // find the closest distance to the left and right and check if
        // there's anything in front
        double minleft = 1e6;
        double minright = 1e6;

        // Get the data from the first sensor of the laser
        const std::vector<meters_t>& scan = sonar->GetSensors()[0].ranges;

        uint32_t sample_count = scan.size();

        for(uint32_t i = 0; i < sample_count; i++){
            //printf("[%s:%d] [robot %d]: %.0f\n", StripPath(__FILE__), __LINE__, id, scan[i]);

            if( (i > (sample_count/4))
            && (i < (sample_count - (sample_count/4)))
            && scan[i] < minfrontdistance)
            {
                //puts( "  obstruction!" );
                obstruction = true;
            }

            if( scan[i] < stopdist )
            {
                //puts( "  stopping!" );
                stop = true;
            }

            if( i > sample_count/2 )
                minleft = std::min( minleft, scan[i] );
            else
                minright = std::min( minright, scan[i] );
        }

            //puts( "" );
            //printf( "minleft %.3f \n", minleft );
            //printf( "minright %.3f\n ", minright );

        if( obstruction || stop || (avoidcount>0) ){
            //printf( "Avoid %d\n", avoidcount );

            pos->SetXSpeed( stop ? 0.0 : avoidspeed );

            /* once we start avoiding, select a turn direction and stick
            with it for a few iterations */
            if( avoidcount < 1 )
            {
                //puts( "Avoid START" );
                avoidcount = random() % avoidduration + avoidduration;

                if( minleft < minright  )
                {
                    pos->SetTurnSpeed( -avoidturn );
                    //printf( "turning right %.2f\n", -avoidturn );
                }
                else
                {
                    pos->SetTurnSpeed( +avoidturn );
                    //printf( "turning left %2f\n", +avoidturn );
                }
            }

            avoidcount--;

            return true; // busy avoding obstacles
        }

        return false; // didn't have to avoid anything
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

        // avoid obstacles
        //robot->ObstacleAvoid();

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
            // remove current path plan
            delete robot->path;
            robot->valid_path = false;

            // log data
            robot->Log();

            // robot needs recharging
            if(robot->state == STATE_GOING_CHARGING){
                // already at docking station
                if(robot->goal == robot->ds.pose){
                    // stop moving
                    pos->Stop();

                    // charging state
                    robot->state = STATE_CHARGE;

                    // subscribe to docking station
                    robot->ds.model->AddCallback(Model::CB_UPDATE, (model_callback_t)ChargingUpdate, robot);
                    robot->ds.model->Subscribe();
                }

                // next goal is docking station
                else if(robot->GoalQueue() && robot->goal_next == robot->ds.pose){
                    robot->SetGoalNext();
                }
            }

            // robot waits for recharging at docking station
            else if(robot->state == STATE_CHARGE_QUEUE){
                // already at docking station
                if(robot->goal == robot->ds.pose){
                    // stop moving
                    pos->Stop();

                    // start docking station auction
                    robot->cord->DockingAuction(pos->GetPose(), robot->ds.id);
                }

                // next goal is docking station
                else if(robot->GoalQueue() && robot->goal_next == robot->ds.pose)
                    robot->SetGoalNext();
            }

            // go to next goal
            else if(robot->GoalQueue()){
                robot->SetGoalNext();
            }

            // continue exploration
            else if(robot->state == STATE_EXPLORE){
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
                if(it->id == robot->ds.id && robot->goal != robot->ds.pose){
                    // move robot to docking station
                    robot->goal.x = it->pose.x;
                    robot->goal.y = it->pose.y;
                    robot->goal.a = it->bearing;
                    robot->SetGoal(robot->goal, 0);
                }
            }
        }

        // robot is currently recharging
        else if(robot->state == STATE_CHARGE){
            // robot is done charging
            if(robot->FullyCharged()){
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
            robot->pos->FindPowerPack()->ChargeStop();
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
