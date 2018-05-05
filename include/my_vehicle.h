//
// Created by Ralf on 5/5/2018.
//

#ifndef PATH_PLANNING_MY_VEHICLE_H
#include "vehicle.h"
#define PATH_PLANNING_MY_VEHICLE_H


class my_vehicle : public vehicle {
private:
    double yaw, speed;
    double previous_yaw, previous_speed;

public:
    my_vehicle();
    void update(double x, double y, double s, double d, double yaw, double speed);

};


#endif //PATH_PLANNING_MY_VEHICLE_H
