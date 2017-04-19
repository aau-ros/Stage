#include "eae_frevo_simple_logoutput.hh"

using namespace Stg;
using namespace std;

namespace eae_frevo_simple
{
    LogOutput::LogOutput(ModelPosition* pos, int robot, string comm, int i_cord, string s_cord, int i_pol, string s_pol, double battery, string map)
    {
        // read from world file
        Worldfile* wf = pos->GetWorld()->GetWorldFile();

        // read number of robots
        int i_robots = 0;
        i_robots = wf->ReadInt(0, "robots", i_robots);

        // read number of docking stations
        int i_dss = 0;
        i_dss = wf->ReadInt(0, "docking_stations", i_dss);

        // read number of current run
        int run = 0;
        run = wf->ReadInt(0, "run", run);

        // read connectivity
        double d_con = 0;
        d_con = wf->ReadFloat(0, "connectivity", d_con);

        // get current time
        std::time_t now = time(NULL);
        struct tm* timeinfo;
        timeinfo = localtime(&now);

        // path to day directory
        char dir[11];
        strftime(dir, 11, "%y-%m-%d/", timeinfo);
        string path = string(getenv("HOME")) + "/" + LOG_PATH + string(dir);

        // create directory if it doesn't exist
        if(MkDir(path) == false)
            return;

        // number of robots for directory name
        ostringstream ss_robots;
        ss_robots << i_robots;

        // number of docking stations for directory name
        ostringstream ss_dss;
        ss_dss << i_dss;

        // coordination strategy index for directory name
        ostringstream ss_cord;
        ss_cord << i_cord;

        // policy index for directory name
        ostringstream ss_pol;
        ss_pol << i_pol;

        // connectivity directory name
        ostringstream ss_con;
        ss_con << d_con;

        // path to specific experiment directory
        path += ss_robots.str() + "-" + ss_dss.str() + "-" + ss_cord.str() + "-" + ss_pol.str() + "-" + ss_con.str() + "/";

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
        output << "# robot\t" << robot << endl << "# number of robots\t" << i_robots << endl << "# number of docking stations\t" << i_dss << endl << "# coordination type\t" << s_cord << endl << "# ds selection policy\t" << s_pol << endl << "# room connectivity\t" << d_con << endl << "# communication type\t" <<  comm << endl << "# battery capacity\t" << battery << endl << "# bitmap\t" << map << endl;
        Write(output.str());
        Write("time\tdistance\tarea\tx position\ty position\tstate\twaiting\tds\tmessages sent\tmessasges received\tbytes sent\tbytes received");
    }

    LogOutput::~LogOutput()
    {
        file.close();
    }

    void LogOutput::Log(usec_t time, double distance, int area, double x, double y, string state, int waiting, int ds, unsigned int msgs_sent, unsigned int msgs_received, unsigned int bytes_sent, unsigned int bytes_received)
    {
        stringstream output;
        output << time/1000000 << "\t" << distance << "\t" << area << "\t" << x << "\t" << y << "\t" << state << "\t" << waiting << "\t" << ds << "\t" << msgs_sent << "\t" << msgs_received << "\t" << bytes_sent << "\t" << bytes_received;
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
