#include "eae_logoutput.hh"

using namespace Stg;
using namespace std;

namespace eae
{
    LogOutput::LogOutput(int robot)
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
        mkdir(path.c_str(), 0777);

        // file name from time
        char timestring[7];
        strftime(timestring, 7, "%H-%M", timeinfo);

        // add robot id to file name
        ostringstream robot_ss;
        robot_ss << robot;

        // complete path of file
        string filepath = path + string(timestring) + "-" + robot_ss.str() + ".log";
        cout << "log file: " << filepath << endl;

        // open log file
        file.open(filepath.c_str());

        // write header
        Write("x position\ty position\tangle\tdistance");
    }

    LogOutput::~LogOutput()
    {
        file.close();
    }

    void LogOutput::Write(string text)
    {
        file << text << endl;
    }
}
