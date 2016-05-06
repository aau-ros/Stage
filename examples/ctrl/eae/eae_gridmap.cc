#include "eae.hh"

using namespace Stg;
using namespace std;

namespace eae
{
    GridMap::GridMap()
    {
        // set dimension parameters
        x_dim = 3;
        y_dim = 3;
        x_offset = 1;
        y_offset = 1;
        x_min = 0-x_offset;
        x_max = 0+x_offset;
        y_min = 0-y_offset;
        y_max = 0+y_offset;

        // fill grid map with unknown values
        vector<char> col(y_dim, unk);
        for(int i=0; i<x_dim; ++i){
            grid.push_back(col);
        }
    }

    void GridMap::Visualize()
    {
        for(unsigned int i=0; i<grid.at(0).size(); ++i){
            for(it = grid.begin(); it<grid.end(); ++it){
                printf("%d ", it->at(i));
            }
            cout << endl;
        }
        cout << endl;
    }

    void GridMap::Insert(int x, int y, char val)
    {
        // x index out of bounds, extend vector in x dimension
        if(x < x_min){
            // number of columns to add
            int add = x_min - x;
            // column containing unknown values
            vector<char> col(y_dim, unk);
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
            vector<char> col(y_dim, unk);
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
                    it->insert(it->begin(), unk);
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
                    it->push_back(unk);
                }
            }
            // adjust dimension parameters
            y_dim += add;
            y_max += add;
        }

        // insert value
        try{
            Write(x, y, val);
        }
        catch(const out_of_range& e){
            printf("Could not write '%d' to (%d,%d)! Something went wrong when extending the map!\n", val, x, y);
        }
    }

    void GridMap::Clear(int x, int y)
    {
        insert(x-1, y-1, fre);
        insert(x-1, y, fre);
        insert(x-1, y+1, fre);
        insert(x, y-1, fre);
        insert(x, y, fre);
        insert(x, y+1, fre);
        insert(x+1, y-1, fre);
        insert(x+1, y, fre);
        insert(x+1, y+1, fre);
    }

    vector< vector <int> > GridMap::Frontiers()
    {
        vector< vector <int> > frontiers;

        int x = x_min;

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

    char GridMap::Read(int x, int y)
    {
        return grid.at(x+x_offset).at(y+y_offset);
    }

    void GridMap::Write(int x, int y, char val)
    {
        grid.at(x+x_offset).at(y+y_offset) = val;
    }

    bool GridMap::Frontier(int x, int y)
    {
        // check cell itself
        try{
            // cell is not free, cannot be frontier
            if(read(x,y) != fre){
                //printf("cell (%d,%d) is not free!\n", x, y);
                return false;
            }
        }
        catch(const out_of_range& e){
            // cell is unknown, cannot be frontier
            //printf("cell (%d,%d) out of range!\n", x, y);
            return false;
        }

        // check neighboring cells
        try{
            // all neighbors are known, cannot be frontier
            if(read(x-1,y-1) != unk && read(x-1,y) != unk && read(x-1,y+1) != unk && read(x,y-1) != unk && read(x,y+1) != unk && read(x+1,y-1) != unk && read(x+1,y) != unk && read(x+1,y+1) != unk){
                //printf("neighbors of cell (%d,%d) are all NOT unknown!\n", x, y);
                return false;
            }
            // one of the neighboring cells is unknown, it is a frontier
            //printf("at least one neighbor of cell(%d,%d) is unknown!\n", x, y);
            return true;
        }
        catch(const out_of_range& e){
            // one of the neighboring cells is unknown, it is a frontier
            //printf("neighbor of cell (%d,%d) out of range!\n", x, y);
            return true;
        }
    }
}
