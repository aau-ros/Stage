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
     * Path for log files. Subdirectories are automatically created.
     */
    const string LOG_PATH = "~/stage_log/";

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
         * @param double battery: The battery capacity.
         */
        LogOutput(int robot, int robots, int dss, string comm, int i_cord, string s_cord, double battery);

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
         */
        void Log(usec_t time, double distance, int area, double x, double y, string state);

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
