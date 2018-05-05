//
// Created by Ralf on 5/5/2018.
//

#ifndef PATH_PLANNING_MY_VEHICLE_H
#include "vehicle.h"
#include "other_vehicles.h"
#define PATH_PLANNING_MY_VEHICLE_H


class my_vehicle : public vehicle {
private:
    other_vehicles &otherVehicles;
    double yaw, speed;
    double previous_yaw, previous_speed;
public:
    my_vehicle(other_vehicles &otherVehicles);
    void update(double x, double y, double s, double d, double yaw, double speed);

    void get_calculated_path();

    bool isInSameLaneAndDirectlyInFrontOfMe();

};


#endif //PATH_PLANNING_MY_VEHICLE_H
