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
