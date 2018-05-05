//
// Created by Ralf on 5/5/2018.
//

#ifndef PATH_PLANNING_OTHER_VEHICLE_H
#include "vehicle.h"
#define PATH_PLANNING_OTHER_VEHICLE_H


class other_vehicle: public vehicle {
private:
    double vx, vy;
    double previous_vx, previous_vy;
public:
    explicit other_vehicle(unsigned int my_id);

    void update(double x, double y, double s, double d, double vx, double vy);

};


#endif //PATH_PLANNING_OTHER_VEHICLE_H
