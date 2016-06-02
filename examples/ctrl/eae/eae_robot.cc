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
        map = new GridMap(pos->GetPose(), pos->GetWorld());
        log = new LogOutput(id);
        cord = new Coordination(pos, this);
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


        /**************
         * update map *
         **************/

        UpdateMap();


        /******************
         * determine goal *
         ******************/

        // initialize goal with current position
        goal = pose;

        Pose frontier;
        double max_bid = BID_INIT;
        double bid;

        // get list of frontiers
        vector< vector <int> > frontiers = map->Frontiers();

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
            if(bid == BID_INV)
                continue;

            // maximize bid, minimize cost
            if(bid > max_bid){
                max_bid = bid;
                goal = frontier;
            }
        }

        // no new goal was found
        if(goal == pose){

            // end of exploration
            if(pos->FindPowerPack()->ProportionRemaining() >= CHARGE_FULL){
                state = STATE_FINISHED;
                printf("exploration finished!\n");
                return;
            }

            // go recharging
            else{
                state = STATE_PRECHARGE;
                goal.x = 0;
                goal.y = 0;
                goal.a = Angle(pose.x, pose.y, 0, 0);
            }
        }


        /********************************
         * coordinate with other robots *
         *******************************/
        cord->FrontierAuction(goal, max_bid);
    }

    void Robot::Move()
    {
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
                printf("invalid goal\n");
                return;
            }
        }

        // store goal for visualization
        pos->waypoints.push_back(ModelPosition::Waypoint(goal, wpcolor));

        // store traveled distance
        dist_travel += pos->GetPose().Distance(goal);

        // move robot to goal
        if(state == STATE_EXPLORE)
            pos->GoTo(goal);

        // move robot to docking station
        else if(state == STATE_PRECHARGE){
            // move to docking station
            if(pos->GetPose().Distance(goal) > 0.25)
                pos->GoTo(goal);

            // creep towards docking station
            else{
                pos->SetXSpeed(0.05);
                pos->SetTurnSpeed(goal.a);
            }

            // stop moving once the recharging starts
            if(pos->FindPowerPack()->GetCharging()){
                pos->Stop();
                state = STATE_CHARGE;
            }
        }

        stringstream output;
        output << goal.x << "\t" << goal.y << "\t" << goal.a << "\t" << dist_travel;
        log->Write(output.str());
    }

    void Robot::Move(Pose to, double bid)
    {
        // robot is at goal, move to next goal
        if(to == goal || pos->GetPose().Distance(goal) < GOAL_TOLERANCE){
            goal = to;
            Move();
        }

        // store as next goal only if my bid is higher than for the other goal already stored
        else if(goal_next_bid == BID_INV || goal_next_bid < bid){
            goal_next = to;
            goal_next_bid = bid;
        }

        else{
            printf("invalid goal or bid!\n");
        }
    }

    double Robot::CalcBid(Pose frontier)
    {
        // calculate distances
        Pose pose = pos->GetPose();
        double dg = pose.Distance(frontier);
        double dgb = Distance(frontier.x, frontier.y, 0, 0);

        // check if frontier is reachable
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

    bool Robot::GoalQueue()
    {
        return goal_next_bid != BID_INV;
    }

    GridMap* Robot::GetMap()
    {
        return map;
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
        return pos->FindPowerPack()->GetStored() / power * velocity;
    }

    int Robot::PositionUpdate(ModelPosition* pos, Robot* robot)
    {
        // initialize robot
        if(robot->state == STATE_INIT){
            //if(pos->GetWorld()->SimTimeNow() > 1000000)
            robot->Init();
        }

        // start exploration
        if(robot->state == STATE_IDLE){
            //if(pos->GetWorld()->SimTimeNow() > 3000000)
            robot->Explore();
        }

        // robot reached goal, continue exploration
        if(robot->state == STATE_EXPLORE && pos->GetPose().Distance(robot->goal) < GOAL_TOLERANCE){
            // there is still a goal in the queue
            if(robot->GoalQueue())
                robot->Move();

            // otherwise just find a new goal
            else
                robot->Explore();
        }

        return 0; // run again
    }

    int Robot::FiducialUpdate(ModelFiducial* fid, Robot* robot)
    {
        // robots is on its way for recharging
        if(robot->state == STATE_PRECHARGE){
            // get all fiducials
            std::vector<ModelFiducial::Fiducial>& fids = fid->GetFiducials();
            std::vector<ModelFiducial::Fiducial>::iterator it;

            // check fiducial return signal
            for(it = fids.begin(); it<fids.end(); ++it){
                // fiducial is docking station
                if(it->id == 2){
                    // move robot to docking station
                    robot->goal.x = it->geom.x;
                    robot->goal.y = it->geom.y;
                    robot->goal.a = it->bearing;
                    robot->Move();
                }
            }
        }

        // robot is currently recharging
        else if(robot->state == STATE_CHARGE){
            // robot is done charging
            if(robot->pos->FindPowerPack()->ProportionRemaining() >= CHARGE_FULL){
                // continue exploration
                robot->state = STATE_EXPLORE;
                robot->Explore();
            }
        }

        return 0; // run again
    }
}
