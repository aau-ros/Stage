#include "frevo_logoutput.hh"

using namespace Stg;
using namespace std;

namespace frevo
{
    LogOutput::LogOutput(ModelPosition* pos, int robot, double battery, string map)
    {
        // read from world file
        Worldfile* wf = pos->GetWorld()->GetWorldFile();

        // read number of current run
        int run = 0;
        run = wf->ReadInt(0, "run", run);

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
        output << "# robot\t" << robot << endl << "# battery capacity\t" << battery << endl << "# bitmap\t" << map << endl;
        Write(output.str());
        Write("time\tdistance\tarea\tx position\ty position\tstate");
    }

    LogOutput::~LogOutput()
    {
        file.close();
    }

    void LogOutput::Log(usec_t time, double distance, int area, double x, double y, string state)
    {
        stringstream output;
        output << time/1000000 << "\t" << distance << "\t" << area << "\t" << x << "\t" << y << "\t" << state;
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
