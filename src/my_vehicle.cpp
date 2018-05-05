//
// Created by Ralf on 5/5/2018.
//

#include "my_vehicle.h"
#include "vehicle.h"

my_vehicle::my_vehicle(): yaw{-1}, speed{-1}, previous_yaw{-1}, previous_speed{-1} {

}

void my_vehicle::update(double x, double y, double s, double d, double yaw, double speed) {
    vehicle::update(x, y, s, d);
    this->previous_yaw = this->yaw;
    this->previous_speed = this->speed;
    this->yaw = yaw;
    this->speed = speed;
}
