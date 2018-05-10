/*
MIT License

Copyright (c) 2018 Ralf Anton Beier

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in all
copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
SOFTWARE.
 */
#ifndef PATH_PLANNING_MY_VEHICLE_H

#include <vector>
#include "vehicle.h"
#include "other_vehicles.h"
#define PATH_PLANNING_MY_VEHICLE_H

typedef enum {
    STATE_IN_LANE,
    STATE_CHANGE_LANE,
} STATE;

class my_vehicle : public vehicle {
private:
    other_vehicles &otherVehicles;
    double yaw;
    double speed;
    double previous_yaw;
    double previous_speed;
    STATE state;
    double ref_vel;
    int target_lane;

    void updateState(const int prev_size, bool &car_in_front_too_close);

    bool isInSameLaneAndDirectlyInFrontOfMe(const int prev_size);
    bool isInLaneAndDirectlyInFrontOfMe(const int prev_size, const unsigned int lane);
    bool isInOtherLaneAndBesidesOfMe(const unsigned int lane);

    bool isInOtherLaneAndDirectlyInBackOfMe(const unsigned int lane);
    bool isLaneFree(const unsigned int lane);

    other_vehicle *getClosestCarInFrontOfMe(unsigned int lane);
    double getTargetSpeed(const int prev_size);

public:
    explicit my_vehicle(other_vehicles &otherVehicles);

    void update(double x, double y, double s, double d, double yaw, double speed);

    void get_calculated_path(const nlohmann::json &previous_path_x, const nlohmann::json &previous_path_y,
                             const double end_path_s, const std::vector<double> &map_waypoints_s,
                             const std::vector<double> &map_waypoints_x,
                             const std::vector<double> &map_waypoints_y, std::vector<double> &next_x_vals,
                             std::vector<double> &next_y_vals);

};


#endif //PATH_PLANNING_MY_VEHICLE_H
