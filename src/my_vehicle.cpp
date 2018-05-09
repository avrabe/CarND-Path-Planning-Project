//
// Created by Ralf on 5/5/2018.
//

#include <vector>
#include <cmath>
#include "spline/spline.h"

#include "my_vehicle.h"

#define LOGURU_WITH_STREAMS 1

#include "loguru/loguru.hpp"

#define PREFERED_LANE 1
#define TARGET_SPEED_MPS 49.5 / 2.236936
#define NUM_LANES 3

// For converting back and forth between radians and degrees.
constexpr double pi() { return M_PI; }

double deg2rad(double x) { return x * pi() / 180; }

// Transform from Frenet s,d coordinates to Cartesian x,y
std::vector<double> getXY(double s, double d, const std::vector<double> &maps_s, const std::vector<double> &maps_x,
                          const std::vector<double> &maps_y) {
    int prev_wp = -1;

    while (s > maps_s[prev_wp + 1] && (prev_wp < (int) (maps_s.size() - 1))) {
        prev_wp++;
    }

    unsigned long wp2 = (prev_wp + 1) % maps_x.size();

    double heading = atan2((maps_y[wp2] - maps_y[prev_wp]), (maps_x[wp2] - maps_x[prev_wp]));
    // the x,y,s along the segment
    double seg_s = (s - maps_s[prev_wp]);

    double seg_x = maps_x[prev_wp] + seg_s * cos(heading);
    double seg_y = maps_y[prev_wp] + seg_s * sin(heading);

    double perp_heading = heading - pi() / 2;

    double x = seg_x + d * cos(perp_heading);
    double y = seg_y + d * sin(perp_heading);

    return {x, y};

}


my_vehicle::my_vehicle(other_vehicles &otherVehicles) : otherVehicles{otherVehicles}, yaw{-1}, speed{-1},
                                                        previous_yaw{-1}, previous_speed{-1}, state{STATE_IN_LANE},
                                                        ref_vel{0}, target_lane{1} {

}

void my_vehicle::update(double x, double y, double s, double d, double yaw, double speed) {
    vehicle::update(x, y, s, d);
    this->previous_yaw = this->yaw;
    this->previous_speed = this->speed;
    this->yaw = yaw;
    this->speed = speed;
    //LOG_S(INFO) << "Mine: speed " << speed << " mp/h, " << speed *1.609344 / 3.6 << " m/s, yaw " << yaw;
}

void my_vehicle::get_calculated_path(const nlohmann::json &previous_path_x, const nlohmann::json &previous_path_y,
                                     const double end_path_s, const double end_path_d,
                                     const std::vector<double> &map_waypoints_s,
                                     const std::vector<double> &map_waypoints_x,
                                     const std::vector<double> &map_waypoints_y,
                                     std::vector<double> &next_x_vals, std::vector<double> &next_y_vals) {

    int prev_size = previous_path_x.size();
    if (prev_size > 0) {
        s = end_path_s;
    }

    bool car_in_front_too_close = false;

    updateState(end_path_s, prev_size, car_in_front_too_close);

    if (car_in_front_too_close && ref_vel > getTargetSpeed(prev_size)) {
        //LOG_S(INFO) << "too close";
        ref_vel -= 0.112;
    }

    std::vector<double> ptsx;
    std::vector<double> ptsy;

    double ref_x = this->x;
    double ref_y = this->y;
    double ref_yaw = deg2rad(this->yaw);

    if (prev_size < 2) {
        double prev_car_x = this->x - cos(this->yaw);
        double prev_car_y = this->y - sin(this->yaw);

        ptsx.push_back(prev_car_x);
        ptsx.push_back(this->x);

        ptsy.push_back(prev_car_y);
        ptsy.push_back(this->y);
    } else {
        ref_x = previous_path_x[prev_size - 1];
        ref_y = previous_path_y[prev_size - 1];

        double ref_x_prev = previous_path_x[prev_size - 2];
        double ref_y_prev = previous_path_y[prev_size - 2];
        ref_yaw = atan2(ref_y - ref_y_prev, ref_x - ref_x_prev);

        ptsx.push_back(ref_x_prev);
        ptsx.push_back(ref_x);

        ptsy.push_back(ref_y_prev);
        ptsy.push_back(ref_y);
    }

    double target_d = lane2d(target_lane);
    std::vector<double> next_wp = getXY(this->s + 40, target_d, map_waypoints_s, map_waypoints_x, map_waypoints_y);
    ptsx.push_back(next_wp[0]);
    ptsy.push_back(next_wp[1]);

    next_wp = getXY(this->s + 80, target_d, map_waypoints_s, map_waypoints_x, map_waypoints_y);
    ptsx.push_back(next_wp[0]);
    ptsy.push_back(next_wp[1]);

    next_wp = getXY(this->s + 120, target_d, map_waypoints_s, map_waypoints_x, map_waypoints_y);
    ptsx.push_back(next_wp[0]);
    ptsy.push_back(next_wp[1]);

    for (unsigned int i = 0; i < ptsx.size(); i++) {
        double shift_x = ptsx[i] - ref_x;
        double shift_y = ptsy[i] - ref_y;

        ptsx[i] = (shift_x * cos(-ref_yaw) - shift_y * sin(-ref_yaw));
        ptsy[i] = (shift_x * sin(-ref_yaw) + shift_y * cos(-ref_yaw));
    }

    tk::spline spline;
    spline.set_points(ptsx, ptsy);

    for (int i = 0; i < prev_size; i++) {
        next_x_vals.push_back(previous_path_x[i]);
        next_y_vals.push_back(previous_path_y[i]);
    }

    double target_x = 30;
    double target_y = spline(target_x);
    double target_dist = sqrt(target_x * target_x + target_y * target_y);

    double x_add_on = 0;
    for (int i = prev_size; i < 50; i++) {
        if (!car_in_front_too_close && ref_vel < getTargetSpeed(prev_size)) {
            ref_vel += 0.112;
        }


        double N = target_dist / (0.02 * ref_vel);
        double x_point = x_add_on + target_x / N;
        double y_point = spline(x_point);

        x_add_on = x_point;

        double x_ref = x_point;
        double y_ref = y_point;

        x_point = x_ref * cos(ref_yaw) - y_ref * sin(ref_yaw);
        y_point = x_ref * sin(ref_yaw) + y_ref * cos(ref_yaw);

        x_point += ref_x;
        y_point += ref_y;

        next_x_vals.push_back(x_point);
        next_y_vals.push_back(y_point);
    }
}

void my_vehicle::updateState(const double end_path_s, const int prev_size,
                             bool &car_in_front_too_close) {
    car_in_front_too_close = false;

    switch (state) {
        case STATE_CHANGE_LANE:
            if (fabs(d - lane2d(target_lane)) < 0.3) {
                state = STATE_IN_LANE;
            }
            car_in_front_too_close = isInLaneAndDirectlyInFrontOfMe(prev_size, target_lane);
            break;
        case STATE_IN_LANE:
            // initialize all lane costs to 0
            std::vector<int> lane_cost(NUM_LANES, 0);


            switch (getLane()) { // Don't switch two lanes at a time
                case 0:
                    lane_cost[2] = 10000;
                    break;
                case 2:
                    lane_cost[0] = 10000;
                    break;
            }

            for (unsigned int i = 0; i < lane_cost.size(); i++) {
                if (isInOtherLaneAndBesidesOfMe(i)) {
                    int own_lane = (i == getLane()) ? -2000 : 0;
                    other_vehicle *other = getClosetCarInFrontOfMe(i);
                    lane_cost[i] += 6000 + own_lane - fabs(other->s - s) * 10;
                }
            }
            for (unsigned int i = 0; i < lane_cost.size(); i++) {
                if (isInOtherLaneAndDirectlyInBackOfMe(prev_size, i)) {
                    int own_lane = (i == getLane()) ? -3000 : 0;
                    lane_cost[i] += 5000 + own_lane;
                }
            }

            for (unsigned int i = 0; i < lane_cost.size(); i++) {
                if (isInLaneAndDirectlyInFrontOfMe(prev_size, i) == true) {
                    lane_cost[i] += 2000;
                }
            }
            // Prefer lanes with no car
            for (unsigned int i = 0; i < lane_cost.size(); i++) {
                if (isLaneFree(i) == false) {
                    lane_cost[i] += 500;
                }
            }


            // Prefer the preferred lane
            for (unsigned int i = 0; i < lane_cost.size(); i++) {
                if (i != PREFERED_LANE) {
                    lane_cost[i] += 10;
                }
            }

            int minimal_lane = std::distance(lane_cost.begin(), std::min_element(lane_cost.begin(), lane_cost.end()));
            if (minimal_lane != getLane()) {
                state = STATE_CHANGE_LANE;
                target_lane = minimal_lane;
                car_in_front_too_close = false;
                LOG_S(INFO)
                << "Switching to lane " << minimal_lane << " offer was " << lane_cost[0] << ", " << lane_cost[1] << ", "
                << lane_cost[2];
            } else {
                car_in_front_too_close = isInSameLaneAndDirectlyInFrontOfMe(prev_size);
            }

            break;
    }
}

bool my_vehicle::isInSameLaneAndDirectlyInFrontOfMe(const int prev_size) {
    return isInLaneAndDirectlyInFrontOfMe(prev_size, getLane());
}

bool my_vehicle::isInLaneAndDirectlyInFrontOfMe(const int prev_size, const unsigned int lane) {
    double other_car_future_s = 0;
    bool ret = false;

    for (auto &x : this->otherVehicles.otherVehicles) {
        other_vehicle other = x.second;

        other_car_future_s = other.s + prev_size * 0.02 * other.getSpeedMPS();

        if (other.getLane() == lane) {
            //LOG_S(INFO) << other.id << " is on the same lane as me " << getLane();
            if ((other_car_future_s > s) && (other_car_future_s - s < 30)) {
                ret = true;
                //LOG_S(INFO) << closest_vehicle_in_front->id << " is closest in front of me with s " << closest_car_future_s << " mine " << s;
            }
        }

    }
    return ret;
}

bool my_vehicle::isInOtherLaneAndDirectlyInBackOfMe(const int prev_size, const unsigned int lane) {
    bool ret = false;

    for (auto &x : this->otherVehicles.otherVehicles) {
        other_vehicle other = x.second;

        if (other.getLane() == lane) {
            //LOG_S(INFO) << other.id << " is on the same lane as me " << getLane();
            if ((other.s < s && other.s + 120 > s && other.getSpeedMPS() > ref_vel)) {
                ret = true;
                LOG_S(INFO)
                << other.id << " is close in back of me " << other.s << " with " << other.getSpeedMPH() << " mine "
                << s;
            }
        }

    }
    return ret;
}


bool my_vehicle::isInOtherLaneAndBesidesOfMe(const unsigned int lane) {
    bool ret = false;

    for (auto &x : this->otherVehicles.otherVehicles) {
        other_vehicle other = x.second;

        if (other.getLane() == lane) {
            //LOG_S(INFO) << other.id << " is on the same lane as me " << getLane();
            if (fabs(other.s - s) < 40) {
                ret = true;
                //LOG_S(INFO) << closest_vehicle_in_front->id << " is closest in front of me with s " << closest_car_future_s << " mine " << s;
            }
        }

    }
    return ret;
}

bool my_vehicle::isLaneFree(const unsigned int lane) {
    bool ret = true;

    for (auto &x : this->otherVehicles.otherVehicles) {
        other_vehicle other = x.second;

        if (other.getLane() == lane) {
            ret = false;
        }

    }
    return ret;
}

other_vehicle *my_vehicle::getClosetCarInFrontOfMe(unsigned int lane) {
    other_vehicle *closest_vehicle_in_front = nullptr;
    for (auto &x : this->otherVehicles.otherVehicles) {
        if (x.second.getLane() == lane) {
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
    //if (closest_vehicle_in_front != nullptr) {
    //    LOG_S(INFO)
    //    << closest_vehicle_in_front->id << " is closest in front of me with vx " << closest_vehicle_in_front->vx
    //    << " and vy " << closest_vehicle_in_front->vy;
    //}
    return closest_vehicle_in_front;
}

double my_vehicle::getTargetSpeed(const int prev_size) {
    other_vehicle *other = nullptr;
    bool in_front_of_us = false;
    double target_speed_mps = TARGET_SPEED_MPS;
    switch (state) {
        case STATE_CHANGE_LANE:
            in_front_of_us = isInLaneAndDirectlyInFrontOfMe(prev_size, target_lane);
            other = getClosetCarInFrontOfMe(target_lane);
            break;
        case STATE_IN_LANE:
            in_front_of_us = isInLaneAndDirectlyInFrontOfMe(prev_size, getLane());
            other = getClosetCarInFrontOfMe(getLane());
            break;
    }

    if (in_front_of_us) {
        target_speed_mps = fmin(other->getSpeedMPS(), TARGET_SPEED_MPS);
        LOG_S(INFO) << "Adjust to speed of front car " << other->getSpeedMPS() << " :: " << target_speed_mps;
    }

    return target_speed_mps;
}
