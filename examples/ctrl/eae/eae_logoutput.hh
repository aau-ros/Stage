#ifndef EAE_LOGOUTPUT_H
#define EAE_LOGOUTPUT_H

#include "eae.hh"

using namespace Stg;
using namespace std;

/**
 * Namespace for energy aware exploration (eae).
 */
namespace eae
{
    /**
     * Folder for log files inside home directory. Create this folder beforehand, subdirectories are automatically created. Keep trailing slash.
     */
    const string LOG_PATH = "stage_log/";

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
         * @param int robot: Id of robot owning this object, used for log file name.
         * @param int robots: The number of robots in this simulation.
         * @param int dss: The number of docking stations in this simulation.
         * @param string comm: The communication model used for the wifi connections.
         * @param int i_cord: The coordination strategy.
         * @param string s_cord: The coordination strategy as a string.
         * @param int i_pol: The policy for selecting docking stations.
         * @param string s_pol: The policy for selecting docking stations as a string.
         * @param double battery: The battery capacity.
         * @param string map: The name of the bitmap of the underlying map.
         */
        LogOutput(int robot, int robots, int dss, string comm, int i_cord, string s_cord, int i_pol, string s_pol, double battery, string map);

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
         * @param int ds: The docking station ID that is currently selected by the robot.
         * @param unsigned int msgs_sent: Number of messages sent over wifi.
         * @param unsigned int msgs_received: Number of messages received over wifi.
         * @param unsigned int bytes_sent: Number of bytes sent over wifi.
         * @param unsigned int bytes_received: Number of bytes received over wifi.
         */
        void Log(usec_t time, double distance, int area, double x, double y, string state, int waiting, int ds, unsigned int msgs_sent, unsigned int msgs_received, unsigned int bytes_sent, unsigned int bytes_received);

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
    };
}

#endif
