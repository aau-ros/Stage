#include "frevo_robot.hh"

using namespace Stg;
using namespace std;

namespace frevo
{
    Robot::Robot(ModelPosition* pos)
    {
        // robot id
        id = pos->GetId();

        // instantiate objects
        this->pos = pos;
        laser = (ModelRanger*)pos->GetChild("ranger:0");
        //map = new GridMap(pos, id);
        //log = new LogOutput(pos, id);
        cam = new OrthoCamera();
        wpcolor = Color(0,1,0); // waypoint color is green

        // robot still needs to be initialized
        state = STATE_INIT;

        // register callback for position updates
        pos->AddCallback(Model::CB_UPDATE, (model_callback_t)PositionUpdate, this);
        pos->Subscribe();

        // store starting point for visualization
        pos->waypoints.push_back(ModelPosition::Waypoint(pos->GetPose(), wpcolor));

        // subscribe to laser ranger to receive data
        laser->AddCallback(Model::CB_UPDATE, (model_callback_t)LidarUpdate, this);
        laser->Subscribe();
        
        // goal for navigation
        goal = pos->GetPose();

        // distance traveled
        dist_travel = 0;
        
        // emergency exits
        vector<int> exit {-24,-16};
        exits.push_back(exit);
        exit = {-16,24};
        exits.push_back(exit);
        exit = {16,-24};
        exits.push_back(exit);
        exit = {24,16};
        exits.push_back(exit);
    }

    Robot::~Robot()
    {
        //delete map;
        //delete log;
        delete cam;
    }

    void Robot::Init()
    {
        if(InArray(id, DEBUG_ROBOTS, sizeof(DEBUG_ROBOTS)/sizeof(id)))
            printf("[%s:%d] [robot %d]: init\n", StripPath(__FILE__), __LINE__, id);

        // pose at last map update
        last_pose = pos->GetPose();

        // init done
        state = STATE_IDLE;
    }
    
    void Robot::Escape()
    {
        // start finding emergency exit
        if(state == STATE_IDLE)
            state = STATE_ESCAPE;
        
        // invalid state
        if(state != STATE_ESCAPE)
            return;
        
        if(InArray(id, DEBUG_ROBOTS, sizeof(DEBUG_ROBOTS)/sizeof(id)))
            printf("[%s:%d] [robot %d]: escape\n", StripPath(__FILE__), __LINE__, id);
        
        // position of robot
        float x = pos->GetPose().x;
        float y = pos->GetPose().y;
        
        if(InArray(id, DEBUG_ROBOTS, sizeof(DEBUG_ROBOTS)/sizeof(id)))
            printf("[%s:%d] [robot %d]: position (%.2f,%.2f)\n", StripPath(__FILE__), __LINE__, id, x, y);
        
        // sense occupancy of neighboring cells
        LidarSense();
        Sense();
        
        if(InArray(id, DEBUG_ROBOTS, sizeof(DEBUG_ROBOTS)/sizeof(id)))
            printf("[%s:%d] [robot %d]: neighbors\n%d %d %d\n%d   %d\n%d %d %d\n\n", StripPath(__FILE__), __LINE__, id, occupancy[3], occupancy[2], occupancy[1], occupancy[4], occupancy[0], occupancy[5], occupancy[6], occupancy[7]);
        
        // calculate distance to closest emergency exit
        float dist = -1;
        float dist_x, dist_y;
        vector<int> closest;
        vector< vector<int> >::iterator it;
        for(it=exits.begin(); it<exits.end(); ++it){
            float dist_cur = hypot(it->at(0)-x, it->at(1)-y);
            if(dist_cur < dist || dist < 0){
                dist = dist_cur;
                closest = *it;
            }
        }
        if(dist > 0){
            dist_x = closest[0] - x;
            dist_y = closest[1] - y;
        }
        
        if(InArray(id, DEBUG_ROBOTS, sizeof(DEBUG_ROBOTS)/sizeof(id))){
            if(dist > 0)
                printf("[%s:%d] [robot %d]: closest exit at (%d,%d)\n", StripPath(__FILE__), __LINE__, id, closest[0], closest[1]);
            else
                printf("[%s:%d] [robot %d]: could not find emergency exit\n", StripPath(__FILE__), __LINE__, id);
        }
        
        
        // inputs for candidate representation (1=free, 0=occupied)
        // input[0] .. horizontal distance between the agent and the nearest emergency exit
        // input[1] .. vertical distance between the agent and the nearest emergency exit
        // input[2] .. field north of the agent is free
        // input[3] .. field north-east of the agent is free
        // input[4] .. field east of the agent is free
        // input[5] .. field south-east of the agent is free
        // input[6] .. field south of the agent is free
        // input[7] .. field south-west of the agent is free
        // input[8] .. field west of the agent is free
        // input[9] .. field north-west of the agent is free

        // outputs of candidate representation
        // output[0] .. horizontal velocity of the agent
        // output[1] .. vertical velocity of the agent

        // ArrayList<Float> output = a.representation.getOutput(input);

        // move agent according to output if that field is free
        Move(new Pose(goal.x+1, goal.y+1, 0, 0));
    }
    
    void Robot::LidarSense()
    {
        vector<meters_t> data = laser->GetSensors()[0].ranges;
        vector<meters_t>::iterator it;
        
        for(it = data.begin(); it<data.end(); ++it){
            if(*it > 0)
                printf("%.2f ", *it);
        }
    }
    
    void Robot::Sense()
    {
        // get map from underlying floorplan model
        Model* map = this->pos->GetWorld()->GetModel("floorplan");

        // read map size
        int w = round(map->GetGeom().size.x);
        int h = round(map->GetGeom().size.y);
        
        if(InArray(id, DEBUG_ROBOTS, sizeof(DEBUG_ROBOTS)/sizeof(id)))
            printf("[%s:%d] [robot %d]: map size %.2fx%.2f\n", StripPath(__FILE__), __LINE__, id, map->GetGeom().size.x, map->GetGeom().size.x);

        // read map origin
        Pose origin = map->GetPose();
        
        if(InArray(id, DEBUG_ROBOTS, sizeof(DEBUG_ROBOTS)/sizeof(id)))
            printf("[%s:%d] [robot %d]: origin (%.2f,%.2f)\n", StripPath(__FILE__), __LINE__, id, origin.x, origin.x);
        
        // coordinates of current position
        int x = floor(pos->GetPose().x) - origin.x + w/2;
        int y = floor(pos->GetPose().y) - origin.y + h/2;
        
        if(InArray(id, DEBUG_ROBOTS, sizeof(DEBUG_ROBOTS)/sizeof(id)))
            printf("[%s:%d] [robot %d]: position (%.2f,%.2f)\n", StripPath(__FILE__), __LINE__, id, pos->GetPose().x, pos->GetPose().y);

        // get map data
        uint8_t* data = new uint8_t[w*h];
        memset(data, 0, sizeof(uint8_t)*w*h);
        map->Rasterize(data, w, h, 1, 1);
        
        for(int m=0; m<h; ++m){
            for(int n=0; n<w; ++n){
                printf("%d ", data[m*w + n]);
            }
            printf("\n");
        }
        
        // index for accessing map
        int i;
        
        // initialize all neighbors to occupied
        for(i=0; i<8; ++i)
            occupancy[i] = 1;

        // read map data east
        i = (y)*w + x+1;
        if(0 <= i && i < w*h)
            occupancy[0] = data[i];

        // read map data north east
        i = (y+1)*w + x+1;
        if(0 <= i && i < w*h)
            occupancy[1] = data[i];

        // read map data north
        i = (y+1)*w + x;
        if(0 <= i && i < w*h)
            occupancy[2] = data[i];

        // read map data north west
        i = (y+1)*w + x-1;
        if(0 <= i && i < w*h)
            occupancy[3] = data[i];

        // read map data west
        i = (y)*w + x-1;
        if(0 <= i && i < w*h)
            occupancy[4] = data[i];

        // read map data south west
        i = (y-1)*w + x-1;
        if(0 <= i && i < w*h)
            occupancy[5] = data[i];

        // read map data south
        i = (y-1)*w + x;
        if(0 <= i && i < w*h)
            occupancy[6] = data[i];

        // read map data south east
        i = (y-1)*w + x+1;
        if(0 <= i && i < w*h)
            occupancy[7] = data[i];
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

    double Robot::Angle(double from_x, double from_y, double to_x, double to_y)
    {
        double x = to_x - from_x;
        double y = to_y - from_y;
        return atan2(y,x);
    }

    void Robot::Log()
    {
        //log->Log(pos->GetWorld()->SimTimeNow(), dist_travel, map->Explored(), goal.x, goal.y, STATE_STRING[state], waiting_time, ds->id, cord->msgs_sent, cord->msgs_received, cord->bytes_sent, cord->bytes_received);
    }

    void Robot::Finalize()
    {
        state = STATE_FINISHED;
        printf("[%s:%d] [robot %d]: finished\n", StripPath(__FILE__), __LINE__, id);

        // stop moving
        pos->Stop();

        // log data
        Log();

        // pause if all other robots finished already
        //if(cord->Finished())
        //    pos->GetWorld()->Stop();

        // simulation finished when all agents reached an emergency exit

        // initialize fitness with number of simulated steps

        // add minimum distance between each agent and nearest emergency exit

        // return fitness value
    }

    void Robot::Move(Pose* goal)
    {
        // move robot towards exit
        if(state == STATE_ESCAPE){
            if(InArray(id, DEBUG_ROBOTS, sizeof(DEBUG_ROBOTS)/sizeof(id)))
                printf("[%s:%d] [robot %d]: going to (%.2f,%.2f)\n", StripPath(__FILE__), __LINE__, id, goal->x, goal->y);

            // store way point for visualization
            pos->waypoints.push_back(ModelPosition::Waypoint(*goal, wpcolor));
            
            this->goal = *goal;
            pos->GoTo(this->goal);
        }
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

        // start finding exit
        if(robot->state == STATE_IDLE){
            robot->Escape();
            return 0;
        }

        // robot reached goal
        if(pos->GetPose().Distance(robot->goal) < GOAL_TOLERANCE){ // euclidean
            // log data
            robot->Log();
            

            // check if agent reached emergency exit

            // continue searching for exit
            if(robot->state == STATE_ESCAPE){
                robot->Escape();
            }

            else{
                printf("[%s:%d] [robot %d]: invalid state: %s\n", StripPath(__FILE__), __LINE__, robot->id, STATE_STRING[robot->state].c_str());
            }

            return 0;
        }

        return 0; // run again
    }

    int Robot::LidarUpdate(ModelRanger* lidar, Robot* robot)
    {
        vector<meters_t> data = lidar->GetSensors()[0].ranges;
        vector<meters_t>::iterator it;
        
        printf("LIDAR");
        
        for(it = data.begin(); it<data.end(); ++it){
            if(*it > 0)
                printf("%.2f ", *it);
        }
        
        return 0; // run again
    }
}
