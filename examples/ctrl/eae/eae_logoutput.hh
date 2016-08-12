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
         */
        LogOutput(int robot, int robots, int dss, string comm, int i_cord, string s_cord, int i_pol, string s_pol, double battery);

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
         */
        void Log(usec_t time, double distance, int area, double x, double y, string state, int waiting, int ds);

        /**
         * Write a string as one line to the log file.
         */
        void Write(string text);

    private:
        /**
         * The file stream for the log file.
         */
        ofstream file;
    };
}

#endif
