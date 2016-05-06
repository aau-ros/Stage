#include "eae.hh"

using namespace Stg;
using namespace std;

namespace eae
{
    LogOutput::LogOutput()
    {
        // get current time
        std::time_t now = time(NULL);
        struct tm* timeinfo;
        timeinfo = localtime(&now);

        // path to log file
        char dir[11];
        strftime(dir, 11, "%y-%m-%d/", timeinfo);
        string path = log_path + string(dir);

        // make directory if it doesn't exist
        mkdir(path.c_str(), 0777);

        // file name
        char filename[7];
        strftime(filename, 7, "%H-%M", timeinfo);

        // complete path of file
        string filepath = path + string(filename) + ".log";
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
