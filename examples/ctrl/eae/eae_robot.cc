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

        // robot is idle
        state = STATE_IDLE;

        // register callback for position updates
        pos->AddCallback(Model::CB_UPDATE, (model_callback_t)PositionUpdate, this);
        pos->Subscribe();

        // register callback for fiducial updates
        fid->AddCallback(Model::CB_UPDATE, (model_callback_t)FiducialUpdate, this);
        fid->Subscribe();

        // store starting point for visualization
        pos->waypoints.push_back(ModelPosition::Waypoint(pos->GetPose(), wpcolor));

        /*World* world_map = pos->GetWorld();
        Model* ground = world_map->GetGround();
        Model::RasterVis* mapvis = new Model::RasterVis();
        pos->AddVisualizer(mapvis, true);
        mapvis.Visualize(ground, cam);*/
        //pos->AddBlockRect(3,3,1,1,1);
        Gl::draw_vector(3, 2, 1);

        // distance traveled
        dist_travel = 0;
    }

    void Robot::Explore()
    {
        // only explore if currently exploring or idle
        if(state != STATE_EXPLORE){
            if(state != STATE_IDLE)
                return;
            state = STATE_EXPLORE;
        }


        /**************
         * update map *
         **************/

        // get current position
        Pose pose = pos->GetPose();

        // mark neighborhood as visited
        map->Clear(round(pose.x), round(pose.y));

        // visualize map progress
        map->VisualizeGui(pose);

        // share map with other robots in range
        cord->BroadcastMap();


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
            if(Distance(0, 0) < GOAL_TOLERANCE){
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
            if(pos->GetPose().Distance(goal) > 0.5)
                pos->GoTo(goal);

            // very close to docking station
            else if(pos->GetPose().Distance(goal) < 0.08){
                printf("almost there...\n");
                // touching, back off a bit
                if(pos->Stalled())
                    pos->SetXSpeed(-0.01);

                // close enough
                else
                    pos->Stop();
            }

            // creep towards docking station
            else{
                printf("creeping\n");
                pos->SetXSpeed(0.02);
                pos->SetTurnSpeed(goal.a);
            }
        }

        stringstream output;
        output << goal.x << "\t" << goal.y << "\t" << goal.a << "\t" << dist_travel;
        log->Write(output.str());
    }

    void Robot::Move(Pose to)
    {
        goal = to;
        Move();
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
        if(pos->FindPowerPack()->ProportionRemaining() > 0.5)
            dgbe = -dgb;
        else
            dgbe = dgb;

        // calculate angular parameter
        double theta = 1/PI * (PI - abs(abs(pose.a - Angle(pose.x, pose.y, frontier.x, frontier.y)) - PI));

        // calculate bid
        return -(W1*dg + W2*dgb + W3*dgbe + W4*theta);
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

    GridMap* Robot::GetMap()
    {
        return map;
    }

    void Robot::UpdateMap(GridMap* map)
    {
        this->map->Update(map, pos->GetPose());
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
        // robot reached goal
        if(pos->GetPose().Distance(robot->goal) < GOAL_TOLERANCE && robot->state == STATE_EXPLORE){
            // continue exploration
            if(robot->state == STATE_EXPLORE)
                robot->Explore();

            // start charging
            else if(robot->state == STATE_PRECHARGE){
                robot->state = STATE_CHARGE;
                printf("now in state charging\n");
            }
        }

        return 0; // run again
    }

    int Robot::FiducialUpdate(ModelFiducial* fid, Robot* robot)
    {
        // make sure robots is on its way for recharging
        if(robot->state != STATE_PRECHARGE)
            return 0;

        // get all fiducials
        std::vector<ModelFiducial::Fiducial>& fids = fid->GetFiducials();
        std::vector<ModelFiducial::Fiducial>::iterator it;

        for(it = fids.begin(); it<fids.end(); ++it){
            // fiducial is docking station
            if(it->id == 2){
                robot->goal.x = it->geom.x;
                robot->goal.y = it->geom.y;
                //robot->goal.a = it->geom.a;
                robot->goal.a = it->bearing;
                robot->Move();
            }
        }

        return 0; // run again
    }
}
