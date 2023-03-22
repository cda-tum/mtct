#pragma once
#include <string>

namespace cda_rail {
    struct Train {
        /**
         * Train struct with relevant properties
         * @param name Name of the train
         * @param length Length of the train (in m)
         * @param max_speed Maximum speed of the train (in m/s)
         * @param acceleration Acceleration of the train (in m/s^2)
         * @param deceleration Deceleration of the train (in m/s^2)
         */

        std::string name;
        double length;
        double max_speed;
        double acceleration;
        double deceleration;
    };
}