//
// Created by Ralf on 5/4/2018.
//

#include "vehicle.h"

vehicle::vehicle(unsigned int my_id) : id{my_id} {

};

vehicle::vehicle() : vehicle(0) {

};

void vehicle::update(double x, double y, double vx, double vy, double s, double d) {
    this->x = x;
    this->y = y;
    this->vx = vx;
    this->vy = vy;
    this->s = s;
    this->d = d;
    return;
};
