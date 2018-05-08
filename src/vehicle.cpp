//
// Created by Ralf on 5/4/2018.
//

#include <cmath>
#include "vehicle.h"

#define LOGURU_WITH_STREAMS 1

#include "loguru/loguru.hpp"

vehicle::vehicle(unsigned int my_id) : previous_x{-1}, previous_y{-1}, previous_s{-1}, previous_d{-1}, id{my_id}, x{-1},
                                       y{-1}, s{-1}, d{-1} {

};

vehicle::vehicle() : vehicle(0) {

};

void vehicle::update(double x, double y, double s, double d) {
    this->previous_x = x;
    this->previous_y = y;
    this->previous_s = s;
    this->previous_d = d;
    this->x = x;
    this->y = y;
    this->s = s;
    this->d = d;
    //LOG_S(INFO) << "Car id: " << this->id << ", s=" << s << ", d="<< d << ", lane="<<this->getLane();
    return;
}


bool vehicle::isInSameLane(vehicle &other) {
    auto myLane = this->getLane();
    auto theirLane = other.getLane();
    return myLane == theirLane;
};

int vehicle::getLane() {
    return round((this->d - 2) / 4);
}

int vehicle::lane2d(int lane) {
    return lane * 4 + 2;
}