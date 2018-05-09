//
// Created by Ralf on 5/5/2018.
//

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
    double yaw, speed;
    double previous_yaw, previous_speed;
    STATE state;
    double ref_vel;
    int target_lane;
    int counter;
public:
    my_vehicle(other_vehicles &otherVehicles);
    void update(double x, double y, double s, double d, double yaw, double speed);

    void get_calculated_path(const nlohmann::json &previous_path_x, const nlohmann::json &previous_path_y,
                             const double end_path_s, const double end_path_d,
                             const std::vector<double> &map_waypoints_s, const std::vector<double> &map_waypoints_x,
                             const std::vector<double> &map_waypoints_y,
                             std::vector<double> &next_x_vals, std::vector<double> &next_y_vals);

    void updateState(const double end_path_s, const int prev_size, bool &car_in_front_too_close);

    bool isInSameLaneAndDirectlyInFrontOfMe(const int prev_size);

    bool isInLaneAndDirectlyInFrontOfMe(const int prev_size, const unsigned int lane);

    bool isInOtherLaneAndBesidesOfMe(const unsigned int lane);

    bool isInOtherLaneAndDirectlyInBackOfMe(const int prev_size, const unsigned int lane);
    bool isLaneFree(const unsigned int lane);

    other_vehicle *getClosetCarInFrontOfMe(unsigned int lane);

    double getTargetSpeed(const int prev_size);
};


#endif //PATH_PLANNING_MY_VEHICLE_H
