#include "eae_gridmap.hh"

using namespace Stg;
using namespace std;

namespace eae
{
    GridMap::GridMap(ModelPosition* pos, int robot)
    {
        // store position model
        this->pos = pos;

        // get required objects from position model
        Pose pose = pos->GetPose();
        World* world = pos->GetWorld();

        // set dimension parameters
        x_dim = 1 + 2 * (LASER_RANGE);
        y_dim = 1 + 2 * (LASER_RANGE);
        x_offset = LASER_RANGE - pose.x;
        y_offset = LASER_RANGE - pose.y;
        x_min = pose.x - (LASER_RANGE);
        x_max = pose.x + (LASER_RANGE);
        y_min = pose.y - (LASER_RANGE);
        y_max = pose.y + (LASER_RANGE);

        // fill grid map with unknown values
        vector<grid_cell_v> col(y_dim, CELL_UNKNOWN);
        for(unsigned int i=0; i<x_dim; ++i){
            grid.push_back(col);
        }

        // initialize visualization models for frontier clusters
        int clusters = sizeof(CC)/sizeof(Color);
        for(int i=0; i<clusters; ++i){
            Model* vis_cluster = new Model(world, NULL, "model", "cluster");
            vis_cluster->SetColor(CC[i]);
            vis_cluster->SetObstacleReturn(0);
            vis_cluster->SetGuiMove(0);
            vis_cluster->ClearBlocks();
            vis_clusters.push_back(vis_cluster);
        }

        // initialize visualization model for free space
        vis_free = new Model(world, NULL, "model", "free");
        vis_free->SetColor(Color(1, 1, 1, 1));
        vis_free->SetObstacleReturn(0);
        vis_free->SetGuiMove(0);
        vis_free->ClearBlocks();

        // initialize visualization model for unknown space
        vis_unknown = new Model(world, NULL, "model", "unknown");
        vis_unknown->SetColor(Color(0, 0, 0, 0.5));
        vis_unknown->SetObstacleReturn(0);
        vis_unknown->SetGuiMove(0);
        vis_unknown->ClearBlocks();

        // initialize visualization model for occupied space
        vis_occupied = new Model(world, NULL, "model", "occupied");
        vis_occupied->SetColor(Color(0, 0, 0, 1));
        vis_occupied->SetObstacleReturn(0);
        vis_occupied->SetGuiMove(0);
        vis_occupied->ClearBlocks();

        // robot id
        this->robot = robot;

        // read resolution from world file
        Worldfile* wf = world->GetWorldFile();
        resolution = wf->ReadFloat(0, "resolution", resolution);

        // no rasterization performed yet
        raster_valid = false;
    }

    GridMap::~GridMap()
    {
        delete vis_free;
        delete vis_unknown;
        delete vis_occupied;
        vector<Model*>::iterator it;
        for(it=vis_clusters.begin(); it<vis_clusters.end(); ++it)
            delete *it;
        vis_clusters.clear();
    }

    void GridMap::Visualize(Pose pose)
    {
        int y = y_max;

        for(int i=grid.at(0).size()-1; i>=0; --i){
            int x = x_min;
            vector< vector<grid_cell_v> >::iterator it;

            for(it = grid.begin(); it<grid.end(); ++it){
                // robot position
                if(abs(x - pose.x) < GOAL_TOLERANCE && abs(y - pose.y) < GOAL_TOLERANCE)
                    printf("R ");

                // origin
                else if(abs(x) < GOAL_TOLERANCE  && abs(y) < GOAL_TOLERANCE)
                    printf("O ");

                // free
                else if(it->at(i) == CELL_FREE)
                    printf("_ ");

                // occupied
                else if(it->at(i) == CELL_OCCUPIED)
                    printf("X ");

                // unknown
                else
                    printf("? ");
                ++x;
            }
            printf("\n");
            --y;
        }
        printf("\n");
    }

    void GridMap::VisualizeGui(Pose pose)
    {
        vis_free->ClearBlocks();
        vis_unknown->ClearBlocks();
        vis_occupied->ClearBlocks();

        int y = y_max;

        for(int i=grid.at(0).size()-1; i>=0; --i){
            int x = x_min;
            vector< vector<grid_cell_v> >::iterator it;

            for(it = grid.begin(); it<grid.end(); ++it){
                // free
                if(it->at(i) == CELL_FREE){
                    vis_free->AddBlockRect(x, y, 1, 1, 0);
                }

                // unknown
                else if(it->at(i) == CELL_UNKNOWN)
                    vis_unknown->AddBlockRect(x, y, 1, 1, 0);

                // occupied
                else if(it->at(i) == CELL_OCCUPIED)
                    vis_occupied->AddBlockRect(x, y, 1, 1, 0);

                // unknown
                else
                    printf("[%s:%d] [robot %d]: Could determine cell type at (%d,%d)!\n", StripPath(__FILE__), __LINE__, robot, x, y);
                ++x;
            }
            --y;
        }

        // visualize frontier clusters
        // clear blocks from previous round
        vector<Model*>::iterator m;
        for(m=vis_clusters.begin(); m<vis_clusters.end(); ++m)
            (*m)->ClearBlocks();

        // current cluster
        Model* vis_cluster;
        unsigned int c = 0;

        // iterate over all clusters
        vector< vector< vector<int> > >::iterator it;
        vector< vector<int> >::iterator jt;
        for(it=cluster_frontiers.begin(); it<cluster_frontiers.end(); ++it){
            // select cluster visualizer
            vis_cluster = vis_clusters.at(c);

            // iterate over cluster visualizers
            ++c;
            if(c >= vis_clusters.size())
                c = 0;

            // add block for every frontier in current cluster
            for(jt=it->begin(); jt<it->end(); ++jt)
                vis_cluster->AddBlockRect(jt->at(0), jt->at(1), 1, 1, 0);
        }
    }

    void GridMap::Insert(int x, int y, grid_cell_v val)
    {
        // iterator
        vector< vector<grid_cell_v> >::iterator it;

        // x index out of bounds, extend vector in x dimension
        if(x < x_min){
            // number of columns to add
            int add = x_min - x;
            // column containing unknown values
            vector<grid_cell_v> col(y_dim, CELL_UNKNOWN);
            // add column
            for(int i=0; i<add; ++i){
                grid.insert(grid.begin(), col);
            }
            // adjust dimension parameters
            x_dim += add;
            x_offset += add;
            x_min -= add;
        }

        // x index out of bounds, extend vector in x dimension
        if(x_max < x){
            // number of columns to add
            int add = x - x_max;
            // column containing unknown values
            vector<grid_cell_v> col(y_dim, CELL_UNKNOWN);
            // add column
            for(int i=0; i<add; ++i){
                grid.push_back(col);
            }
            // adjust dimension parameters
            x_dim += add;
            x_max += add;
        }

        // y index out of bounds, extend vector in y dimension
        if(y < y_min){
            // number of elements to add to both sides of each column
            int add = y_min - y;
            // add elements to every column
            for(it=grid.begin(); it<grid.end(); ++it){
                for(int i=0; i<add; ++i){
                    it->insert(it->begin(), CELL_UNKNOWN);
                }
            }
            // adjust dimension parameters
            y_dim += add;
            y_offset += add;
            y_min -= add;
        }

        // y index out of bounds, extend vector in y dimension
        if(y_max < y){
            // number of elements to add to both sides of each column
            int add = y - y_max;
            // add elements to every column
            for(it=grid.begin(); it<grid.end(); ++it){
                for(int i=0; i<add; ++i){
                    it->push_back(CELL_UNKNOWN);
                }
            }
            // adjust dimension parameters
            y_dim += add;
            y_max += add;
        }

        // insert value
        try{
            Write(x,y,val);
        }
        catch(const out_of_range& e){
            printf("[%s:%d] [robot %d]: Could not write '%d' to (%d,%d)! Something went wrong when extending the map!\n", StripPath(__FILE__), __LINE__, robot, val, x, y);
        }
    }

    GridMap* GridMap::Clear(Pose pos, vector<meters_t> scan)
    {
        // local map containing only the updated cells (for returning)
        GridMap* local = new GridMap(this->pos, robot);

        // get global from underlying floorplan model
        Model* map = this->pos->GetWorld()->GetModel("floorplan");

        // read global map size
        int w = round(map->GetGeom().size.x);
        int h = round(map->GetGeom().size.y);

        // read global map origin
        Pose origin = map->GetPose();

        // read global map data
        uint8_t* data = new uint8_t[w*h];
        memset(data, 0, sizeof(uint8_t)*w*h);
        map->Rasterize(data, w, h, 1, 1);

        // robot direction in degree
        int ra = (int)round(pos.a/PI*180);

        // iterate over all angles of fov
        for(int a=ra-135; a<=ra+135; ++a){
            // iterate until laser range or obstacle
            for(int r=0; r<LASER_RANGE; ++r){
                // coordinates in local map
                int x = floor(pos.x) + floor(r*cos((double)a/180*PI));
                int y = floor(pos.y) + floor(r*sin((double)a/180*PI));

                // index in global map
                int i = (y-origin.y+h/2)*w + x-origin.x+w/2;

                // index out of bounds
                if(i < 0 || w*h <= i)
                    break;

                // read value of grid cell in global map
                uint8_t cell = data[i];
                grid_cell_v value;
                switch(cell){
                    case 0:
                        value = CELL_FREE;
                        break;
                    case 1:
                        value = CELL_OCCUPIED;
                        break;
                    default:
                        value = CELL_UNKNOWN;
                        printf("[%s:%d] [robot %d]: Could determine cell value at (%d,%d) with index %d!\n", StripPath(__FILE__), __LINE__, robot, x, y, i);
                }

                // insert cell value in local map
                Insert(x,y,value);
                local->Insert(x,y,value);

                // hit obstacle, stop
                if(value == CELL_OCCUPIED)
                    break;
            }
        }

        delete data;

        return local;
    }

    vector< vector <int> > GridMap::ReachableFrontierClusters(int range)
    {
        vector< vector<int> >::iterator it, jt, kt;
        vector< vector< vector<int> > >::iterator cit, cjt;

        // get all reachable frontiers
        vector< vector<int> > frontiers = ReachableFrontiers(range);

        // reset clusters
        cluster_frontiers.clear();
        cluster_frontiers.reserve(int(ceil(double(frontiers.size())/2.0/LASER_RANGE)));

        // temporary vector
        vector< vector<int> > todo;


        /*********************
         * cluster frontiers *
         *********************/

        // do as long as there are unclustered frontiers
        while(frontiers.size() > 0){
            // pick one frontier
            it = frontiers.begin();

            // create cluster from that frontier
            vector< vector<int> > cluster;
            cluster.push_back(*it);

            // frontiers in new cluster need to be checked with all remaining frontiers
            todo.push_back(*it);

            // remove frontier from vector of frontiers
            frontiers.erase(it);

            // iterate newly added frontiers and check their neighbors
            while(todo.size() > 0){
                // pick one frontier
                vector<int> frontier = todo.front();

                // remove this frontier from todo vector
                todo.erase(todo.begin());

                // iterate all unclustered frontiers
                for(it=frontiers.end()-1; it>=frontiers.begin(); --it){
                    // connecting vector between frontiers
                    vector <int> dir;
                    dir.push_back(frontier[0]-it->at(0));
                    dir.push_back(frontier[1]-it->at(1));

                    // frontiers within clustering distance
                    double dist = hypot(dir.at(0), dir.at(1));

                    if(dist <= FRONTIER_CLUSTER_DIST){
                        // check if no wall is in between
                        bool clear = true;
                        for(int i=1; i<floor(dist); ++i){
                            // calculate coordinates of intermediate cells
                            int x = frontier[0] + floor(double(dir.at(0)) / dist * i);
                            int y = frontier[1] + floor(double(dir.at(1)) / dist * i);

                            // only free cells allowed
                            if(Read(x,y) != CELL_FREE){
                                clear = false;
                                break;
                            }
                        }

                        // frontiers are connected
                        if(clear){
                            // store frontier in cluster
                            cluster.push_back(*it);
                            todo.push_back(*it);

                            // delete from frontiers
                            frontiers.erase(it);
                        }
                    }
                }
            }

            // store cluster in global vector
            cluster_frontiers.push_back(cluster);
        }


        /************************
         * split large clusters *
         ************************/

        // temporary cluster of sorted frontiers
        vector< vector<int> > cluster;
        cluster.reserve(2*LASER_RANGE);

        // vector of temporary clusters
        vector< vector< vector<int> > > cluster_frontiers_temp;

        // iterate over clusters
        for(cit=cluster_frontiers.begin(); cit<cluster_frontiers.end(); ++cit){
            // cluster is too large
            if(cit->size() > 2*LASER_RANGE){
                while(cit->size() > 0){
                    // pick one frontier
                    it = cit->begin();

                    // remove any frontiers from temp cluster
                    cluster.clear();

                    // go along chain of connected frontiers into one direction
                    int d = 1;
                    while(d > 0){
                        // add current frontier to temp cluster
                        cluster.push_back(*it);
                        // and remove it from original cluster
                        cit->erase(it);

                        // temp cluster reached its maximum size
                        if(cluster.size() >= 2*LASER_RANGE){
                            break;
                        }

                        // find closest neighbor
                        d = -1;
                        for(jt=cit->begin(); jt<cit->end(); ++jt){
                            int d_temp = hypot(jt->at(0)-cluster.back().at(0), jt->at(1)-cluster.back().at(1));
                            if(d_temp <= FRONTIER_CLUSTER_DIST && (d_temp < d || d < 0)){
                                d = d_temp;
                                it = jt;
                            }
                        }
                    }

                    // temp cluster has still room
                    if(cluster.size() < 2*LASER_RANGE){
                        // go along chain of connected frontiers into OTHER direction
                        while(true){
                            // find closest neighbor
                            d = -1;
                            for(jt=cit->begin(); jt<cit->end(); ++jt){
                                int d_temp = hypot(jt->at(0)-cluster.front().at(0), jt->at(1)-cluster.front().at(1));
                                if(d_temp <= FRONTIER_CLUSTER_DIST && (d_temp < d || d < 0)){
                                    d = d_temp;
                                    it = jt;
                                }
                            }

                            // no neighbor found, cluster finished
                            if(d <= 0){
                                break;
                            }

                            // add current frontier to temp cluster
                            cluster.insert(cluster.begin(), *it);
                            //cluster.push_back(*it);
                            // and remove it from original cluster
                            cit->erase(it);

                            // temp cluster reached its maximum size
                            if(cluster.size() >= 2*LASER_RANGE){
                                break;
                            }
                        }
                    }

                    // add temp cluster to clusters
                    cluster_frontiers_temp.push_back(cluster);
                }
            }
        }

        // remove empty clusters
        for(cit=cluster_frontiers.end()-1; cit>=cluster_frontiers.begin(); --cit){
            if(cit->size() <= 0)
                cluster_frontiers.erase(cit);
        }

        // add temporary clusters to global vector
        for(cit=cluster_frontiers_temp.begin(); cit<cluster_frontiers_temp.end(); ++cit){
            cluster_frontiers.push_back(*cit);
        }


        /**************************
         * calculate spatial mean *
         **************************/

        // reset clusters
        clusters.clear();

        // iterate over all clusters to calculate spatial mean
        for(cit=cluster_frontiers.begin(); cit<cluster_frontiers.end(); ++cit){
            // summed coordinates of frontiers in cluster
            int x = 0;
            int y = 0;

            // sum all coordinates of frontiers in cluster
            for(it=cit->begin(); it<cit->end(); ++it){
                x += it->at(0);
                y += it->at(1);
            }

            // calculate spatial mean
            vector<int> cluster;
            cluster.push_back(round(double(x)/cit->size()));
            cluster.push_back(round(double(y)/cit->size()));
            clusters.push_back(cluster);
        }

        return clusters;
    }

    vector< vector <int> > GridMap::ReachableFrontiers(int range, int max_frontiers)
    {
        vector< vector<int> > frontiers;

        // unique coordinates in range
        set< vector<int> > coords;
        pair< set< vector<int> >::iterator,bool> ret;

        // number of frontiers
        int num_frontiers = 0;

        // number of angles to search
        double angles;

        // search with increasing radius
        for(int r=1; r<range; ++r){
            // set number of angles per quater circle
            angles = r*3; // approximation

            // search all angles
            for(double a=0; a<2*PI; a+=PI/2.0/angles){
                // calculate cartesian coordinates
                double xd = double(r)*cos(a);
                double yd = double(r)*sin(a);
                vector<int> coord;
                if(xd > 0)
                    coord.push_back(floor(xd));
                else
                    coord.push_back(ceil(xd));
                if(yd > 0)
                    coord.push_back(floor(yd));
                else
                    coord.push_back(ceil(yd));

                // store unique coordinates
                ret = coords.insert(coord);

                // got new unique coordinates
                // check if frontier is there
                if(ret.second == true && Frontier(coord[0],coord[1])){
                    vector<int> frontier;
                    frontier.push_back(coord[0]);
                    frontier.push_back(coord[1]);
                    frontiers.push_back(frontier);
                    ++num_frontiers;

                    // only find max_frontiers frontiers
                    if(max_frontiers > 0 && num_frontiers >= max_frontiers)
                        return frontiers;
                }
            }
        }

        return frontiers;
    }

    vector< vector <int> > GridMap::Frontiers()
    {
        vector< vector <int> > frontiers;

        int x = x_min;
        vector< vector<grid_cell_v> >::iterator it;
        vector<grid_cell_v>::iterator jt;

        for(it = grid.begin(); it<grid.end(); ++it){
            int y = y_min;
            for(jt = it->begin(); jt<it->end(); ++jt){
                if(Frontier(x,y)){
                    vector<int> frontier;
                    frontier.push_back(x);
                    frontier.push_back(y);
                    frontiers.push_back(frontier);
                }
                ++y;
            }
            ++x;
        }

        return frontiers;
    }

    void GridMap::Update(GridMap* map)
    {
        int x = map->x_min;
        vector< vector<grid_cell_v> >::iterator it;
        vector<grid_cell_v>::iterator jt;

        for(it = map->grid.begin(); it<map->grid.end(); ++it){
            int y = map->y_min;
            for(jt = it->begin(); jt<it->end(); ++jt){
                // write new value if cell is unknown
                try{
                    if(Read(x,y) == CELL_UNKNOWN){
                        Write(x,y,*jt);
                    }
                }
                // extend map and insert new value if coordinates are out of range
                catch(const out_of_range& e){
                    Insert(x,y,*jt);
                }
                ++y;
            }
            ++x;
        }
    }

    int GridMap::ExploredCells()
    {
        int explored = 0;
        vector< vector<grid_cell_v> >::iterator it;
        vector<grid_cell_v>::iterator jt;

        for(it = grid.begin(); it<grid.end(); ++it){
            for(jt = it->begin(); jt<it->end(); ++jt){
                if(*jt != CELL_UNKNOWN){
                    ++explored;
                }
            }
        }
        return explored;
    }

    double GridMap::Explored()
    {
        // multiply twice with resolution because we have a two-dimensional area
        return ExploredCells() * resolution * resolution;
    }

    int GridMap::Distance(double from_x, double from_y, double to_x, double to_y)
    {
        // execute a* algorithm
        vector<ast::point_t> path;
        if(AStar(Pose(from_x, from_y, 0, 0), Pose(to_x, to_y, 0, 0), &path) == false)
            return -1;

        return (int)path.size();
    }

    bool GridMap::AStar(Pose start_pose, Pose goal_pose, vector<ast::point_t>* path)
    {
        // start and goal
        grid_cell_t start_cell = M2C(start_pose);
        grid_cell_t goal_cell = M2C(goal_pose);
        ast::point_t start(start_cell.x, start_cell.y);
        ast::point_t goal(goal_cell.x, goal_cell.y);

        // find path
        bool result = ast::astar(Rasterize(), (uint32_t)Width(), (uint32_t)Height(), start, goal, *path);

        // error
        if(!result){
            return false;
        }

        return true;
    }

    grid_cell_t GridMap::M2C(Pose m)
    {
        // scale
        m.x /= resolution;
        m.y /= resolution;

        // quantize
        grid_cell_t c;
        c.x= (unsigned int)round(m.x);
        c.y= (unsigned int)round(m.y);

        // shift
        c.x += x_offset;
        c.y += y_offset;

        return c;
    }

    Pose GridMap::C2M(grid_cell_t c)
    {
        // shift
        c.x -= x_offset;
        c.y -= y_offset;

        // convert
        Pose m;
        m.x= (meters_t)c.x;
        m.y= (meters_t)c.y;

        // scale
        m.x *= resolution;
        m.y *= resolution;

        return m;
    }

    double GridMap::C2M(unsigned int c)
    {
        // convert
        double m = (double)c;

        // scale
        m *= resolution;

        return m;
    }

    unsigned int GridMap::Width()
    {
        return x_dim;
    }

    unsigned int GridMap::Height()
    {
        return y_dim;
    }

    unsigned int GridMap::Size()
    {
        return sizeof(this) + Width() * Height() * sizeof(grid_cell_v);
    }

    uint8_t* GridMap::Rasterize()
    {
        // valid raster existent
        if(raster_valid == false){
            // new raster needs to be created
            raster = new uint8_t[x_dim*y_dim];
            int idx = 0;

            for(unsigned int i=0; i<grid.at(0).size(); ++i){
                vector< vector<grid_cell_v> >::iterator it;

                for(it = grid.begin(); it<grid.end(); ++it){
                    // free
                    if(it->at(i) == CELL_FREE){
                        raster[idx] = 1;
                    }

                    // occupied
                    else if(it->at(i) == CELL_OCCUPIED){
                        raster[idx] = 9;
                    }

                    // unknown
                    else{
                        if(PLAN_UNKNOWN){
                            raster[idx] = 1;
                        }
                        else{
                            raster[idx] = 9;
                        }
                    }
                    ++idx;
                }
            }

            // make raster valid
            raster_valid = true;
        }

        return raster;
    }

    grid_cell_v GridMap::Read(int x, int y)
    {
        return grid.at(x+x_offset).at(y+y_offset);
    }

    void GridMap::Write(int x, int y, grid_cell_v val)
    {
        // only write if value changed
        if(grid.at(x+x_offset).at(y+y_offset) != val){
            grid.at(x+x_offset).at(y+y_offset) = val;
            // invalidate rasterized map
            if(raster_valid){
                raster_valid = false;
                delete raster;
            }
        }
    }

    bool GridMap::Frontier(int x, int y)
    {
        // check cell itself
        try{
            // cell is not free, cannot be frontier
            if(Read(x,y) != CELL_FREE){
                return false;
            }
        }
        catch(const out_of_range& e){
            // cell is unknown, cannot be frontier
            return false;
        }

        // check neighboring cells
        try{
            // one neighbors is occupied, robot cannot go so close to walls
            if(Read(x-1,y) == CELL_OCCUPIED || Read(x,y-1) == CELL_OCCUPIED || Read(x,y+1) == CELL_OCCUPIED || Read(x+1,y) == CELL_OCCUPIED){
                return false;
            }
            // all neighbors are known, cannot be frontier
            if(Read(x-1,y) != CELL_UNKNOWN && Read(x,y-1) != CELL_UNKNOWN && Read(x,y+1) != CELL_UNKNOWN && Read(x+1,y) != CELL_UNKNOWN){
                return false;
            }
            // one of the neighboring cells is unknown, it is a frontier
            return true;
        }
        catch(const out_of_range& e){
            // one of the neighboring cells is unknown, it is a frontier
            return true;
        }
    }
}
