#include "eae_logoutput.hh"

using namespace Stg;
using namespace std;

namespace eae
{
    LogOutput::LogOutput(int robot, int robots, int dss, string comm, int i_cord, string s_cord, double battery)
    {
        // get current time
        std::time_t now = time(NULL);
        struct tm* timeinfo;
        timeinfo = localtime(&now);

        // path to log file
        char dir[11];
        strftime(dir, 11, "%y-%m-%d/", timeinfo);
        string path = LOG_PATH + string(dir);

        // make directory if it doesn't exist
        if(mkdir(path.c_str(), 0777) < 0){
            printf("could not create log folder, nothing will be recorded!\n");
            return;
        }

        // file name from time
        char timestring[7];
        strftime(timestring, 7, "%H-%M", timeinfo);

        // add coordination strategy index to file name
        ostringstream ss_cord;
        ss_cord << i_cord;

        // add robot id to file name
        ostringstream ss_robot;
        ss_robot << robot;

        // complete path of file
        string filepath = path + string(timestring) + "-" + ss_cord.str() + "-" + ss_robot.str() + ".log";
        printf("log file: %s\n", filepath.c_str());

        // open log file
        file.open(filepath.c_str());

        // write header
        stringstream output;
        output << "# robot\t" << robot << endl << "# number of robots\t" << robots << endl << "# number of docking stations\t" << dss << endl <<"# communication type\t" <<  comm << endl << "# coordination type\t" << s_cord << endl << "# battery capacity\t" << battery << endl;
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
}
