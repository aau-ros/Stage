#include "swarm_logoutput.hh"

using namespace Stg;
using namespace std;

namespace swarm
{
    LogOutput::LogOutput(ModelPosition* pos, int robot, double battery, string map)
    {
        // save robot id
        this->robot = robot;
        
        // read from world file
        Worldfile* wf = pos->GetWorld()->GetWorldFile();

        // read simulation identifier
        string s_simulation = "";
        s_simulation = wf->ReadString(0, "simulation", s_simulation);

        // read number of robots
        int i_robots = 0;
        i_robots = wf->ReadInt(0, "robots", i_robots);

        // read number of docking stations
        int i_dss = 0;
        i_dss = wf->ReadInt(0, "docking_stations", i_dss);

        // read number of current run
        int run = 0;
        run = wf->ReadInt(0, "run", run);

        // path to log directory
        string path = string(getenv("HOME")) + "/" + LOG_PATH;

        // create directory if it doesn't exist
        if(MkDir(path) == false)
            return;

        // number of robots for directory name
        ostringstream ss_robots;
        ss_robots << i_robots;

        // number of docking stations for directory name
        ostringstream ss_dss;
        ss_dss << i_dss;

        // path to specific experiment directory
        path += ss_robots.str() + "-" + ss_dss.str() + "-" + s_simulation + "/";

        // create directory if it doesn't exist
        if(MkDir(path) == false)
            return;

        // add run number to file name
        ostringstream ss_run;
        ss_run << run;

        // add robot id to file name
        ostringstream ss_robot;
        ss_robot << robot;

        // complete path of file
        string filepath = path + ss_run.str() + "-" + ss_robot.str() + ".log";
        printf("[Log file \"%s\"]\n", filepath.c_str());

        // backup old log file if exists
        Backup(filepath);

        // open log file
        file.open(filepath.c_str());

        // write header
        stringstream output;
        output << "# robot\t" << robot << endl << "# number of robots\t" << i_robots << endl << "# number of docking stations\t" << i_dss << endl << "# battery capacity\t" << battery << endl << "# bitmap\t" << map << endl;
        Write(output.str());
        Write("time\tdistance\tarea\tx position\ty position\tstate\twaiting\tmap width\tmap height\tmap origin x\tmap origin y\tmap");
    }

    LogOutput::~LogOutput()
    {
        file.close();
    }

    void LogOutput::Log(usec_t time, double distance, int area, double x, double y, string state, int waiting, GridMap* map)
    {
        stringstream output;
        
        // get map
        int width = map->Width();
        int height = map->Height();
        int* origin = map->Origin();
        uint8_t* gridmap = map->Output();
        
        // output measurements
        output << time/1000000 << "\t" << distance << "\t" << area << "\t" << x << "\t" << y << "\t" << state << "\t" << waiting << "\t" << width << "\t" << height << "\t" << origin[0] << "\t" << origin[1] << "\t";
        
        // output map
        if(InArray(robot, DEBUG_ROBOTS, sizeof(DEBUG_ROBOTS)/sizeof(robot)))
            printf("[%s:%d] [robot %d]: logging map of size (%d,%d)\n", StripPath(__FILE__), __LINE__, robot, width, height);
        for(int i=0; i<width*height; ++i){
            output << (int)gridmap[i];
        }
        
        Write(output.str());
    }

    void LogOutput::Write(string text)
    {
        file << text << endl;
    }

    bool LogOutput::MkDir(string path)
    {
        struct stat sb;
        if(stat(path.c_str(), &sb) != 0 || S_ISDIR(sb.st_mode) == false){
            if(mkdir(path.c_str(), 0755) < 0){
                string base_path = string(getenv("HOME")) + "/" + LOG_PATH;
                printf("Could not create log folder %s, please create it manually!\n", base_path.c_str());
                return false;
            }
        }
        return true;
    }

    void LogOutput::Backup(string path)
    {
        // get current time
        std::time_t now = time(NULL);
        struct tm* timeinfo;
        timeinfo = localtime(&now);

        // file info
        struct stat sb;

        // file exists
        if(stat(path.c_str(), &sb) == 0){
            // create new file name with time
            char timestring[7];
            strftime(timestring, 7, "%H-%M", timeinfo);
            string backup = path + "." + string(timestring);

            printf("[Backup previous log file to \"%s\"]\n", backup.c_str());

            // rename file
            rename(path.c_str(), backup.c_str());
        }
    }
}
