#ifndef SWARM_LOGOUTPUT_H
#define SWARM_LOGOUTPUT_H

#include "swarm.hh"
#include "swarm_gridmap.hh"

using namespace Stg;
using namespace std;

/**
 * Namespace for energy aware exploration (swarm).
 */
namespace swarm
{
    /**
     * Folder for log files inside home directory. Create this folder beforehand, subdirectories are automatically created. Keep trailing slash.
     */
    const string LOG_PATH = "Programs/Stage/log/";

    /**
     * A class for logging data.
     */
    class LogOutput
    {
    public:
        /**
         * Constructor.
         * Creates a subdirectory in the LOG_PATH directory, according to current date.
         *
         * @param ModelPosition* pos: The instantiated position model of the robot.
         * @param int robot: Id of robot owning this object, used for log file name.
         * @param double battery: The battery capacity.
         * @param string map: The name of the bitmap of the underlying map.
         */
        LogOutput(ModelPosition* pos, int robot, double battery, string map);

        /**
         * Destructor.
         * Closes the log file.
         */
        ~LogOutput();

        /**
         * Write a line with the current status to the log file.
         *
         * @param usec_t time: The current simulation time.
         * @param double distance: The total distance traveled by the robot so far.
         * @param int area: The area explored collectively by all robots so far.
         * @param double x: The current location of the robot (x-coordinate).
         * @param double y: The current location of the robot (y-coordinate).
         * @param string state: The current state of the robot.
         * @param int waiting: The accumulated waiting time at docking stations.
         * @param GridMap* map: Map explored by the robot.
         */
        void Log(usec_t time, double distance, int area, double x, double y, string state, int waiting, GridMap* map);

        /**
         * Write a string as one line to the log file.
         */
        void Write(string text);

    private:
        /**
         * Create a directory.
         *
         * @param string path: The path of the directory to create.
         *
         * @return bool: False if the directory could not be created, true otherwise.
         */
        bool MkDir(string path);

        /**
         * Backup a file by renaming it. The hour and minute are added to the end.
         *
         * @param string path: Path to the file.
         */
        void Backup(string path);

        /**
         * The file stream for the log file.
         */
        ofstream file;

        /**
         * Robot ID.
         */
        int robot;
    };
}

#endif
