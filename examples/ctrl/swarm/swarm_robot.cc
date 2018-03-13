#include "swarm_robot.hh"

extern "C" {
#include "candidate.c"
}

using namespace Stg;
using namespace std;

namespace swarm
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
        log = new LogOutput(pos, id, pos->FindPowerPack()->GetCapacity(), MapName());
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

        // timeout for retrying exploration
        to_exp = 0;

        // countdown for unsuccessful exploration tries
        cd_exp = CD_EXP;

        // TODO: Obstacle avoidance
        /*avoidcount = 0;
        randcount = 0;*/
    }

    Robot::~Robot()
    {
        delete map;
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

        // initial logging
        Log();

        // init done
        state = STATE_IDLE;
    }

    void Robot::Explore()
    {
        // finish exploration after countdown
        if(cd_exp <= 0){
            Finalize();
            return;
        }

        // only explore with a given rate
        if(to_exp > pos->GetWorld()->SimTimeNow()){
            return;
        }

        // only explore if currently exploring or idle
        if(state != STATE_EXPLORE){
            if(state != STATE_IDLE)
                return;
            state = STATE_EXPLORE;
        }

        if(InArray(id, DEBUG_ROBOTS, sizeof(DEBUG_ROBOTS)/sizeof(id)))
            printf("[%s:%d] [robot %d]: exploring\n", StripPath(__FILE__), __LINE__, id);


        /************************************
         * determine goal                   *
         * use evolved candidate controller *
         ************************************/

        // candidate inputs
        float in[8];

        // robot battery charge percentage [0,1]
        //in[0] = robot->pos->FindPowerPack()->ProportionRemaining();
        // obstacle density in all sectors
        in[0] = ObstacleDensity(0);
        in[1] = ObstacleDensity(1);
        in[2] = ObstacleDensity(2);
        in[3] = ObstacleDensity(3);
        // robot density in all sectors
        in[4] = RobotDensity(0);
        in[5] = RobotDensity(1);
        in[6] = RobotDensity(2);
        in[7] = RobotDensity(3);
        // charging point density in all sectors
        // battery charge

        if(InArray(id, DEBUG_ROBOTS, sizeof(DEBUG_ROBOTS)/sizeof(id))){
            printf("[%s:%d] [robot %d]: neural network inputs\n", StripPath(__FILE__), __LINE__, id);
            printf("[%s:%d] [robot %d]: obstacle densities: [%.2f, %.2f, %.2f, %.2f]\n", StripPath(__FILE__), __LINE__, id, in[0], in[1], in[2], in[3]);
            printf("[%s:%d] [robot %d]: robot densities: [%.2f, %.2f, %.2f, %.2f]\n", StripPath(__FILE__), __LINE__, id, in[4], in[5], in[6], in[7]);
        }

        // use candidate to get coordinates for next goal
        Result out = getOutput(in, 8);

        // initialize goal with curren pose
        Pose goal = pos->GetPose();

        // calculate coordinates of goal from given outputs
        try{
            if(InArray(id, DEBUG_ROBOTS, sizeof(DEBUG_ROBOTS)/sizeof(id))){
                printf("[%s:%d] [robot %d]: neural network output: [%.2f, %.2f, %.2f, %.2f]\n", StripPath(__FILE__), __LINE__, id, out.output[0], out.output[1], out.output[2], out.output[3]);
            }

            // angular offset between sectors and x/y axes
            radians_t a = -PI / 4;

            // rotated x/y values from candidate outputs
            meters_t xr = (laser->GetSensors()[0].range.max * 0.5) * (out.output[0] - out.output[2]);
            meters_t yr = (laser->GetSensors()[0].range.max * 0.5) * (out.output[1] - out.output[3]);

            // rotate outputs to x/y axes
            goal.x += xr * cos(a) - yr * sin(a);
            goal.y += xr * sin(a) + yr * cos(a);
        }
        catch(const out_of_range& e){
            printf("[%s:%d] [robot %d]: neural network output has wrong size! shutting down...\n", StripPath(__FILE__), __LINE__, id);
            Finalize();
        }

        // set timeout after which next exploration can happen
        to_exp = pos->GetWorld()->SimTimeNow() + TO_EXP;

        // goal found, move there
        SetGoal(goal);
    }

    void Robot::Move()
    {
        // don't move if charging, dead or done exploring
        if(state == STATE_CHARGE || state == STATE_DEAD || state == STATE_FINISHED)
            return;

        // move robot to frontier / docking station
        if(state == STATE_EXPLORE || state == STATE_GOING_CHARGING || state == STATE_CHARGE_QUEUE){
            // go to next waypoint
            pos->GoTo(int_goal);
        }
    }

    void Robot::SetGoal(Pose to)
    {
        // go to new goal
        goal = to;

        // make a new plan
        if(InArray(id, DEBUG_ROBOTS, sizeof(DEBUG_ROBOTS)/sizeof(id)))
            printf("[%s:%d] [robot %d]: moving on new path to (%.2f,%.2f)\n", StripPath(__FILE__), __LINE__, id, goal.x, goal.y);

        // plan path to goal
        valid_path = Plan(pos->GetPose(), goal);

        // no plan found
        if(!valid_path){
            if(InArray(id, DEBUG_ROBOTS, sizeof(DEBUG_ROBOTS)/sizeof(id)))
                printf("[%s:%d] [robot %d]: cannot reach (%.2f,%.2f)\n", StripPath(__FILE__), __LINE__, id, goal.x, goal.y);

            // continue countdown
            --cd_exp;

            // turn robot, then try again to find a goal
            Pose rpos = pos->GetPose();
            rpos.a = normalize(rpos.a + MAP_UPDATE_ANGLE);
            pos->GoTo(rpos);

            return;
        }

        // reset countdown
        cd_exp = CD_EXP;

        // store goal for visualization
        pos->waypoints.push_back(ModelPosition::Waypoint(goal, wpcolor));

        // next intermediate goal
        int_goal = path->PopBack()->pose;

        // move towards goal
        Move();
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

    GridMap* Robot::GetMap()
    {
        return map;
    }

    int Robot::Distance(double to_x, double to_y)
    {
        return map->Distance(pos->GetPose().x, pos->GetPose().y, to_x, to_y);
    }

    bool Robot::SamePoint(Pose point1, Pose point2)
    {
        return point1.Distance(point2) <= EPSILON; // euclidean
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
        map->Clear(pose, laser->GetSensors()[0].ranges);

        // visualize map progress
        //map->VisualizeGui(pose);
        //map->Visualize(pose);
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
        // TODO: log message statistics?
        log->Log(pos->GetWorld()->SimTimeNow(), dist_travel, map->Explored(), goal.x, goal.y, STATE_STRING[state], waiting_time, map);
    }

    void Robot::Finalize()
    {
        state = STATE_FINISHED;
        printf("[%s:%d] [robot %d]: exploration finished\n", StripPath(__FILE__), __LINE__, id);

        // stop moving
        pos->Stop();

        // log data
        Log();
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

    float Robot::ObstacleDensity(int sector)
    {
        // get lidar sensor object
        ModelRanger::Sensor lidar = laser->GetSensors()[0];

        // get current laser scan samples
        vector<meters_t> scan = lidar.ranges;
        vector<meters_t>::iterator it;

        // portion of scan that belongs to a specific sector
        vector<meters_t>::iterator begin = scan.begin() + sector * lidar.sample_count / SECTORS;
        vector<meters_t>::iterator end = scan.begin() + (sector + 1) * lidar.sample_count / SECTORS;

        // measure free space
        float free = 0;
        int test_count = 0;

        for(it = begin; it < end && it < scan.end(); ++it){
            ++test_count;
            // clip lidar to valid ranges
            if(*it < 0)
                free += 0;
            else if(lidar.range.max < *it)
                free += (float)lidar.range.max;
            else
                free += *it;
        }

        // return density
        return 1 - free / ((end - begin) * (float)lidar.range.max);
    }

    float Robot::RobotDensity(int sector)
    {
        return cord->RobotDensity(sector * 2*PI / SECTORS, (sector + 1) * 2*PI / SECTORS);
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

        // continue exploration
        if(robot->valid_path == false){
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
        }

        // robot reached goal
        if(dist > GOAL_TOLERANCE && pos->GetPose().Distance(robot->goal) < GOAL_TOLERANCE){ // euclidean
            // stop moving
            pos->Stop();

            // remove current path plan
            if(robot->valid_path){
                delete robot->path;
                robot->valid_path = false;
            }

            // clear map, in case the two goals where closer than MAP_UPDATE_DIST
            if(!map_update)
                robot->UpdateMap();

            // log data
            robot->Log();

            // continue exploration
            robot->Explore();

            return 0;
        }

        // robot reached intermediate goal
        if(robot->valid_path && pos->GetPose().Distance(robot->int_goal) < GOAL_TOLERANCE){ // euclidean
            // next intermediate goal
            Node* node = robot->path->PopBack();

            // valid intermediate goal
            if(node != NULL)
                robot->int_goal = node->pose;

            // invalid path
            else{
                delete robot->path;
                robot->valid_path = false;

                // clear map, in case the two goals where closer than MAP_UPDATE_DIST
                if(!map_update)
                    robot->UpdateMap();

                // log data
                robot->Log();

                // continue exploration
                robot->Explore();

                return 0;
            }

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
        /*
        // stop recharging
        if(robot->pos->FindPowerPack()->ProportionRemaining() >= CHARGE_UNTIL || robot->state != STATE_CHARGE){
            robot->pos->FindPowerPack()->ChargeStop();
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
        */

        return 0; // run again
    }
}
