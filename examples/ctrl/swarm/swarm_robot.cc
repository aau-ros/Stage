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
        // obstacle density in all directions
        in[0] = ObstacleDensity(0);
        in[1] = ObstacleDensity(1);
        in[2] = ObstacleDensity(2);
        in[3] = ObstacleDensity(3);
        // robot density in all directions
        in[4] = RobotDensity(0);
        in[5] = RobotDensity(1);
        in[6] = RobotDensity(2);
        in[7] = RobotDensity(3);
        // charging point density in all directions
        // battery charge

        if(InArray(id, DEBUG_ROBOTS, sizeof(DEBUG_ROBOTS)/sizeof(id))){
            printf("[%s:%d] [robot %d]: neural network inputs\n", StripPath(__FILE__), __LINE__, id);
            printf("[%s:%d] [robot %d]: obstacle densities: [%.2f, %.2f, %.2f, %.2f]\n", StripPath(__FILE__), __LINE__, id, in[0], in[1], in[2], in[3]);
        }
        
        // use candidate to get goal direction
        Result out = getOutput(in, 8);
        double dir = out.output[0] * PI;
        
        // initialize goal with curren pose
        Pose goal = pos->GetPose();
        
        // calculate coordinates of goal in given direction
        goal.x += (laser->GetSensors()[0].range.max / 2) * sin(dir);
        goal.y += (laser->GetSensors()[0].range.max / 2) * cos(dir);

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
            // go on a straight line towards goal
            //pos->GoTo(goal);
            
            // current pose
            Pose pose = pos->GetPose();

            // get angular velocity to reach next node on path
            double a_goal, a_error;
            path->GoodDirection(pose, 2, a_goal);
            a_error = normalize(a_goal - pose.a);

            // set velocities and avoid obstacles
            SetMotorSpeed(a_error);
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
        // try to find new, reachable goal
        if(!valid_path){
            if(InArray(id, DEBUG_ROBOTS, sizeof(DEBUG_ROBOTS)/sizeof(id)))
                printf("[%s:%d] [robot %d]: cannot reach (%.2f,%.2f)\n", StripPath(__FILE__), __LINE__, id, goal.x, goal.y);
            
            // finish simulation
            Finalize();
            
            // set goal to current pose so that exploration continues
            //goal = pos->GetPose();
            
            return;
        }

        // store goal for visualization
        pos->waypoints.push_back(ModelPosition::Waypoint(goal, wpcolor));

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
        log->Log(pos->GetWorld()->SimTimeNow(), dist_travel, map->Explored(), goal.x, goal.y, STATE_STRING[state], waiting_time, map);//, map->Rasterize(), map->Width(), map->Height());
    }

    void Robot::Finalize()
    {
        state = STATE_FINISHED;
        printf("[%s:%d] [robot %d]: exploration finished\n", StripPath(__FILE__), __LINE__, id);

        // stop moving
        pos->Stop();

        // log data
        Log();

        // pause simulation
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
    
    double Robot::ObstacleDensity(int direction)
    {
        // get lidar sensor object
        ModelRanger::Sensor lidar = laser->GetSensors()[0];
        
        // get current laser scan samples
        vector<meters_t> scan = lidar.ranges;
        vector<meters_t>::iterator it;
        
        // portion of scan that belongs to a specific direction
        vector<meters_t>::iterator begin = scan.begin() + direction * lidar.sample_count / DIRECTIONS;
        vector<meters_t>::iterator end = scan.begin() + (direction + 1) * lidar.sample_count / DIRECTIONS - 1;
        
        // count occupied samples
        double occupied = 0;
        for(it = begin; it <= end && it < scan.end(); ++it){
            if(*it < lidar.range.max)
                ++occupied;
        }
        
        // return density
        return occupied / (lidar.sample_count / DIRECTIONS);
    }
    
    double Robot::RobotDensity(int direction)
    {
        // TODO: also description in header file
        return 0.0;
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

        // robot is following the path to the next step, continue moving
        if(robot->valid_path){
            robot->Move();
            return 0;
        }

        return 0; // run again
    }

    int Robot::FiducialUpdate(ModelFiducial* fid, Robot* robot)
    {
        /*
        // get all fiducials
        vector<ModelFiducial::Fiducial>& fids = fid->GetFiducials();
        vector<ModelFiducial::Fiducial>::iterator it;

        // store docking stations
        for(it = fids.begin(); it<fids.end(); ++it){
            robot->cord->UpdateDs(it->id, it->pose);
        }
        */

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
