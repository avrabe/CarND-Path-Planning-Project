//
// Created by Ralf on 5/5/2018.
//

#include "my_vehicle.h"

#define LOGURU_WITH_STREAMS 1

#include "loguru/loguru.hpp"

my_vehicle::my_vehicle(other_vehicles &otherVehicles) : otherVehicles{otherVehicles}, yaw{-1}, speed{-1},
                                                        previous_yaw{-1}, previous_speed{-1} {

}

void my_vehicle::update(double x, double y, double s, double d, double yaw, double speed) {
    vehicle::update(x, y, s, d);
    this->previous_yaw = this->yaw;
    this->previous_speed = this->speed;
    this->yaw = yaw;
    this->speed = speed;
}

void my_vehicle::get_calculated_path() {
    isInSameLaneAndDirectlyInFrontOfMe();
}

bool my_vehicle::isInSameLaneAndDirectlyInFrontOfMe() {
    other_vehicle *closest_vehicle_in_front = nullptr;
    for (auto &x : this->otherVehicles.otherVehicles) {
        if (isInSameLane(x.second)) {
            //LOG_S(INFO) << x.second.id << " is on the same lane as me " << getLane();
            if (this->s > x.second.s) {
                //LOG_S(INFO) << x.second.id << " is in back of me ";
            } else if (closest_vehicle_in_front == nullptr) {
                //LOG_S(INFO) << x.second.id << " is in front of me ";
                closest_vehicle_in_front = static_cast<other_vehicle *>(&x.second);
            } else if (closest_vehicle_in_front->s > x.second.s) {
                //LOG_S(INFO) << x.second.id << " is closer in front of me than " << closest_vehicle_in_front->id;
                closest_vehicle_in_front = static_cast<other_vehicle *>(&x.second);
            }
        }

    }
    if (closest_vehicle_in_front != nullptr) {
        LOG_S(INFO)
        << closest_vehicle_in_front->id << " is closest in front of me with vx " << closest_vehicle_in_front->vx
        << " and vy " << closest_vehicle_in_front->vy;
    }
    return false;
}
