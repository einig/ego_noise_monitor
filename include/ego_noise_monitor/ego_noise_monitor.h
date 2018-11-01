//! ego_noise_monitor/ego_noise_monitor monitors the noise created by the robot itself and the location of the sound source
/*!
*/
#ifndef _EGO_NOISE_MONITOR_H_
#define _EGO_NOISE_MONITOR_H_
#include "ros/ros.h"

class EgoNoiseMonitor {
    public:
        //! Constructor
        /*!  */
        EgoNoiseMonitor();
        ~EgoNoiseMonitor();

        void Test();

    private:
};

#endif