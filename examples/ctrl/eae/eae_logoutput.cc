#include "eae_logoutput.hh"

using namespace Stg;
using namespace std;

namespace eae
{
    LogOutput::LogOutput(int robot, int robots, int dss, string comm, int i_cord, string s_cord, int i_pol, string s_pol, double battery, string map)
    {
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

        // coordination strategy index for directory name
        ostringstream ss_cord;
        ss_cord << i_cord;

        // policy index directory name
        ostringstream ss_pol;
        ss_pol << i_pol;

        // number of robots for directory name
        ostringstream ss_robots;
        ss_robots << robots;

        // path to specific experiment directory
        path += ss_cord.str() + "-" + ss_pol.str() + "-" + ss_robots.str() + "/";

        // create directory if it doesn't exist
        if(MkDir(path) == false)
            return;

        // add map number to file name
        string map_no = map.substr(map.rfind("_")+1, map.rfind(".")-map.rfind("_")-1);

        // add robot id to file name
        ostringstream ss_robot;
        ss_robot << robot;

        // complete path of file
        string filepath = path + map_no + "-" + ss_robot.str() + ".log";
        printf("[Log file \"%s\"]\n", filepath.c_str());

        // backup old log file if exists
        Backup(filepath);

        // open log file
        file.open(filepath.c_str());

        // write header
        stringstream output;
        output << "# robot\t" << robot << endl << "# number of robots\t" << robots << endl << "# number of docking stations\t" << dss << endl <<"# communication type\t" <<  comm << endl << "# coordination type\t" << s_cord << endl << "# ds selection policy\t" << s_pol << endl << "# battery capacity\t" << battery << endl << "# bitmap\t" << map << endl;
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
