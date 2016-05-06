#include "eae.hh"

using namespace Stg;
using namespace std;

namespace eae
{
    Robot::Robot(ModelPosition* pos) : pos(pos)
    {
        // instantiate objects
        map = new GridMap();
        log = new LogOutput();
        cam = new OrthoCamera();
        wpcolor = Color(0,1,0); // waypoint color is green

        // robot is idle
        state = STATE_IDLE;

        // at the start the previous position equals the current position
        prev_pose = pos->GetPose();

        // register callback for position updates
        pos->AddCallback(Model::CB_UPDATE, (model_callback_t)PositionUpdate, this );
        pos->Subscribe();

        // initialize wifi adapter
        wifi = (ModelWifi*)pos->GetChild("wifi:0");
        wifi->AddCallback(Model::CB_UPDATE, (model_callback_t)WifiUpdate, this);
        wifi->comm.SetReceiveMsgFn(ProcessMessage);
        wifi->Subscribe();

        // store starting point for visualization
        pos->waypoints.push_back(ModelPosition::Waypoint(0, 0, 0, 0, wpcolor));

        /*World* world_map = pos->GetWorld();
        Model* ground = world_map->GetGround();
        pos->AddVisualizer(&mapvis, true);
        mapvis.Visualize(ground, cam);*/

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
        map->clear(round(pose.x), round(pose.y));
        //map->visualize();


        /*******************************
         * estimate remaining distance *
         *******************************/

        double velocity; // average velocity
        double power;    // power consumption at average velocity
        double dist_rem; // remaining distance that the robot can drive with its current battery
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
        dist_rem = pos->FindPowerPack()->GetStored() / power * velocity;

        //printf("power: %.1f, dist: %.1f\n", power, dist_rem);


        /******************
         * determine goal *
         ******************/

        // initialize goal with current position
        goal = pose;

        // parameters for cost function
        double dg;    // distance to frontier
        double dgb;   // distance from frontier to home base
        double dgbe;  // energy-aware distance from frontier to home base
        double theta; // required turning of robot to reach frontier
        double cost;  // total cost
        double min_cost = 200; // initialize cost

        // frontier location
        int x,y;

        // get list of frontiers
        vector< vector <int> > frontiers = map->Frontiers();

        // iterate through all frontiers
        vector< vector<int> >::iterator it;
        for(it=frontiers.begin(); it<frontiers.end(); ++it){
            // frontier location
            x = it->at(0);
            y = it->at(1);

            // compute distances
            dg = Distance(x, y);
            dgb = Distance(x, y, 0, 0);

            // not enough energy to reach frontier
            if(dist_rem <= dg + dgb)
                continue;

            // energy-aware distance
            if(pos->FindPowerPack()->ProportionRemaining() > 0.5)
                dgbe = -dgb;
            else
                dgbe = dgb;

            // compute required turning
            theta = 1/pi * (pi - abs(abs(Angle(prev_pose.x, prev_pose.y, pose.x, pose.y) - Angle(pose.x, pose.y, x, y)) - pi));

            // compute total cost
            cost = w1*dg + w2*dgb + w3*dgbe + w4*theta;

            // minimize cost
            if(cost < min_cost){

                /*printf("coordinate:\n");
                printf("x: %d\n", x);
                printf("y: %d\n\n", y);
                printf("cost function:\n");
                printf("dg:    %.2f\n", dg);
                printf("dgb:   %.2f\n", dgb);
                printf("dgbe:  %.2f\n", dgbe);
                printf("theta: %.2f\n", theta);
                printf("\n");*/

                min_cost = cost;
                goal.x = x;
                goal.y = y;
                goal.a = Angle(pose.x, pose.y, x, y);
            }
        }

        // no new goal was found
        if(goal == pose){

            // end of exploration
            if(Distance(0, 0) < goal_tolerance){
                state = STATE_FINISHED;
                printf("exploration finished!\n");
                return;
            }

            // go recharging
            else{
                goal.x = 0;
                goal.y = 0;
                goal.a = Angle(pose.x, pose.y, 0, 0);
            }
        }


        /********************************
         * coordinate with other robots *
         *******************************/

        if(cord->FrontierAuction(goal, min_cost) == false){
            Explore();
            return;
        }


        /****************
         * move to goal *
         ****************/

        // store goal for visualization
        pos->waypoints.push_back(ModelPosition::Waypoint(goal, wpcolor));

        // store traveled distance
        dist_travel += pose.Distance(goal);

        // store previous position
        prev_pose = pose;

        // move robot
        pos->GoTo(goal);

        stringstream output;
        output << goal.x << "\t" << goal.y << "\t" << goal.a << "\t" << dist_travel;
        log->write(output.str());
        //printf("move to (%.0f,%.0f,%.1f), cost %.1f\n\n\n", goal.x, goal.y, goal.a, min_cost);
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

    int Robot::PositionUpdate(ModelPosition* pos, Robot* robot)
    {
        // robot reached goal, continue exploration
        if(pos->GetPose().Distance(robot->goal) < goal_tolerance && robot->state == STATE_EXPLORE){
            robot->Explore();
        }
        //printf("velocity:  x=%.1f, y=%.1f, z=%.1f, a=%.1f\n", pos->GetVelocity().x, pos->GetVelocity().y, pos->GetVelocity().z, pos->GetVelocity().a);

        return 0; // run again
    }
}
