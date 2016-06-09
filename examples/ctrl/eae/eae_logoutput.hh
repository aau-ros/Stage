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
    const string LOG_PATH = "/media/mrappapo/Daten/Arbeit/NES/projects/2014_mrs/simulation/cpp_dock-coord/";

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
         */
        LogOutput(int robot);

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
         * @param robot_state_t state: The current state of the robot.
         */
        void Log(usec_t time, double distance, int area, double x, double y, robot_state_t state);

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
