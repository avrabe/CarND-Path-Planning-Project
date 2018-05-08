//
// Created by Ralf on 5/5/2018.
//
#include <cmath>
#include "other_vehicle.h"

other_vehicle::other_vehicle(unsigned int my_id) : vehicle(my_id), previous_vx{-1}, previous_vy{-1}, vx{-1}, vy{-1} {

}

void other_vehicle::update(double x, double y, double s, double d, double vx, double vy) {
    vehicle::update(x, y, s, d);
    this->previous_vx = this->vx;
    this->previous_vy = this->vy;
    this->vx = vx;
    this->vy = vy;
}

double other_vehicle::getSpeedMPH() {
    return std::sqrt(std::pow(this->vx, 2) + std::pow(this->vy, 2));
}

double other_vehicle::getSpeedMPS() {
    return getSpeedMPH() / 2.236936;

}


