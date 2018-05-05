//
// Created by Ralf on 5/5/2018.
//

#include "other_vehicle.h"

other_vehicle::other_vehicle(unsigned int my_id) : vehicle(my_id), vx{-1}, vy{-1}, previous_vx{-1}, previous_vy{-1} {

}

void other_vehicle::update(double x, double y, double s, double d, double vx, double vy) {
    vehicle::update(x, y, s, d);
    this->previous_vx = vx;
    this->previous_vy = vy;
    this->vx = vx;
    this->vy = vy;
}
