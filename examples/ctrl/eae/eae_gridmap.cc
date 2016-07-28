#include "eae_gridmap.hh"

using namespace Stg;
using namespace std;

namespace eae
{
    GridMap::GridMap(Pose pose, World* world, int robot)
    {
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

        // initialize visualization model for frontiers
        vis_frontier = new Model(world, NULL, "model", "frontier");
        vis_frontier->SetObstacleReturn(0);
        vis_frontier->SetColor(Color(0, 1, 0, 0.5));
        vis_frontier->ClearBlocks();
        vis_frontier->SetGuiMove(0);

        // initialize visualization model for free space
        vis_free = new Model(world, NULL, "model", "free");
        vis_free->SetObstacleReturn(0);
        vis_free->SetColor(Color(1, 1, 1, 1));
        vis_free->ClearBlocks();
        vis_free->SetGuiMove(0);

        // initialize visualization model for unknown space
        vis_unknown = new Model(world, NULL, "model", "unknown");
        vis_unknown->SetObstacleReturn(0);
        vis_unknown->SetColor(Color(0, 0, 0, 0.5));
        vis_unknown->ClearBlocks();
        vis_unknown->SetGuiMove(0);

        // initialize visualization model for occupied space
        vis_occupied = new Model(world, NULL, "model", "occupied");
        vis_occupied->SetObstacleReturn(0);
        vis_occupied->SetColor(Color(0, 0, 0, 1));
        vis_occupied->ClearBlocks();
        vis_occupied->SetGuiMove(0);

        // robot id
        this->robot = robot;

        // read resolution from world file
        Worldfile* wf = world->GetWorldFile();
        resolution = wf->ReadFloat(0, "resolution", resolution);
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
        vis_frontier->ClearBlocks();
        vis_free->ClearBlocks();
        vis_unknown->ClearBlocks();
        vis_occupied->ClearBlocks();

        int y = y_max;

        for(int i=grid.at(0).size()-1; i>=0; --i){
            int x = x_min;
            vector< vector<grid_cell_v> >::iterator it;

            for(it = grid.begin(); it<grid.end(); ++it){
                // frontier
                if(Frontier(x,y))
                    vis_frontier->AddBlockRect(x, y, 1, 1, 0);

                // free
                else if(it->at(i) == CELL_FREE)
                    vis_free->AddBlockRect(x, y, 1, 1, 0);

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
        // local map (for returning)
        GridMap* local = new GridMap(pos, vis_frontier->GetWorld(), robot);

        // number of scan values
        int n = scan.size();
        int j = 0;

        // iterate through all scan values, i.e. all scan angles
        vector<meters_t>::iterator it;
        for(it=scan.begin(); it<scan.end(); ++it){
            // absolute scan angle (starting from pos x-asis)
            double a = pos.a - LASER_FOV/2 + (j+0.5)/n*LASER_FOV;
            ++j;

            // absolute cell indexes
            int x = 0;
            int y = 0;

            // raytrace along the scan angle and set cells to free
            int i;
            for(i=0; i<*it; ++i){
                x = pos.x + round(i*cos(a));
                y = pos.y + round(i*sin(a));
                Insert(x,y,CELL_FREE);
                local->Insert(x,y,CELL_FREE);
            }

            // if ray hit an obstacle, set cell to occupied
            if(i<LASER_RANGE){
                x = pos.x + round(i*cos(a));
                y = pos.y + round(i*sin(a));
                Insert(x,y,CELL_OCCUPIED);
                local->Insert(x,y,CELL_OCCUPIED);
            }
        }

        return local;
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

    GridMap& GridMap::operator=(const GridMap& toCopy)
    {
        grid = toCopy.grid;

        x_dim = toCopy.x_dim;
        y_dim = toCopy.y_dim;
        x_offset = toCopy.x_offset;
        y_offset = toCopy.y_offset;
        x_min = toCopy.x_min;
        x_max = toCopy.x_max;
        y_min = toCopy.y_min;
        y_max = toCopy.y_max;

        vis_frontier = toCopy.vis_frontier;
        vis_free = toCopy.vis_free;
        vis_unknown = toCopy.vis_unknown;
        vis_occupied = toCopy.vis_occupied;

        return *this;
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

    uint8_t* GridMap::Rasterize()
    {
        uint8_t* data = new uint8_t[x_dim*y_dim];
        int idx = 0;

        for(unsigned int i=0; i<grid.at(0).size(); ++i){
            vector< vector<grid_cell_v> >::iterator it;

            for(it = grid.begin(); it<grid.end(); ++it){
                // free
                if(it->at(i) == CELL_FREE){
                    data[idx] = 1;
                }

                // occupied
                else if(it->at(i) == CELL_OCCUPIED){
                    data[idx] = 9;
                }

                // unknown
                else{
                    if(PLAN_UNKNOWN){
                        data[idx] = 1;
                    }
                    else{
                        data[idx] = 9;
                    }
                }
                ++idx;
            }
        }

        return data;
    }

    grid_cell_v GridMap::Read(int x, int y)
    {
        return grid.at(x+x_offset).at(y+y_offset);
    }

    void GridMap::Write(int x, int y, grid_cell_v val)
    {
        grid.at(x+x_offset).at(y+y_offset) = val;
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
            // all neighbors are known, cannot be frontier
            if(Read(x-1,y-1) != CELL_UNKNOWN && Read(x-1,y) != CELL_UNKNOWN && Read(x-1,y+1) != CELL_UNKNOWN && Read(x,y-1) != CELL_UNKNOWN && Read(x,y+1) != CELL_UNKNOWN && Read(x+1,y-1) != CELL_UNKNOWN && Read(x+1,y) != CELL_UNKNOWN && Read(x+1,y+1) != CELL_UNKNOWN){
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
