/*
MIT License

Copyright (c) 2018 Ralf Anton Beier

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in all
copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
SOFTWARE.
 */
#include <vector>
#include <cmath>
#include "spline/spline.h"

#include "my_vehicle.h"

#define LOGURU_WITH_STREAMS 1

#include "loguru/loguru.hpp"

#define PREFERRED_LANE 1
#define TARGET_SPEED_MPS 49.5 / 2.236936
#define NUMBER_OF_LANES 3

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
}

void my_vehicle::get_calculated_path(const nlohmann::json &previous_path_x, const nlohmann::json &previous_path_y,
                                     const double end_path_s, const std::vector<double> &map_waypoints_s,
                                     const std::vector<double> &map_waypoints_x,
                                     const std::vector<double> &map_waypoints_y, std::vector<double> &next_x_vals,
                                     std::vector<double> &next_y_vals) {

    unsigned int prev_size = previous_path_x.size();
    if (prev_size > 1) {
        s = end_path_s;
    }

    bool car_in_front_too_close = false;

    updateState(prev_size, car_in_front_too_close);

    double speed = getTargetSpeed(prev_size);

    std::vector<double> points_x;
    std::vector<double> points_y;

    double ref_x = this->x;
    double ref_y = this->y;
    double ref_yaw = deg2rad(this->yaw);

    if (prev_size < 2) {
        double prev_car_x = this->x - cos(this->yaw);
        double prev_car_y = this->y - sin(this->yaw);

        points_x.push_back(prev_car_x);
        points_x.push_back(this->x);

        points_y.push_back(prev_car_y);
        points_y.push_back(this->y);
    } else {
        ref_x = previous_path_x[prev_size - 1];
        ref_y = previous_path_y[prev_size - 1];

        double ref_x_prev = previous_path_x[prev_size - 2];
        double ref_y_prev = previous_path_y[prev_size - 2];
        ref_yaw = atan2(ref_y - ref_y_prev, ref_x - ref_x_prev);

        points_x.push_back(ref_x_prev);
        points_x.push_back(ref_x);

        points_y.push_back(ref_y_prev);
        points_y.push_back(ref_y);
    }

    double target_d = lane2d(target_lane);
    unsigned int point = 0;
    std::vector<double> next_wp;
    std::vector<unsigned int> steps;

    switch (state) {
        case STATE_CHANGE_LANE:
            // first step will change the lane.
            steps = {40, 60, 90};
            break;
        case STATE_IN_LANE:
            // Try to stay in lane
            steps = {12, 15, 20, 25, 30, 60, 90};
            break;
    }

    for (auto &x : steps) {
        next_wp = getXY(this->s + x, target_d, map_waypoints_s, map_waypoints_x, map_waypoints_y);
        points_x.push_back(next_wp[0]);
        points_y.push_back(next_wp[1]);
    }

    for (point = 0; point < points_x.size(); point++) {
        double shift_x = points_x[point] - ref_x;
        double shift_y = points_y[point] - ref_y;

        points_x[point] = (shift_x * cos(-ref_yaw) - shift_y * sin(-ref_yaw));
        points_y[point] = (shift_x * sin(-ref_yaw) + shift_y * cos(-ref_yaw));
    }

    tk::spline spline;
    spline.set_points(points_x, points_y);

    // The remains of the previous path will be the next next path
    for (point = 0; point < prev_size; point++) {
        next_x_vals.push_back(previous_path_x[point]);
        next_y_vals.push_back(previous_path_y[point]);
    }

    double target_x = 20;
    double target_y = spline(target_x);
    double target_dist = sqrt(target_x * target_x + target_y * target_y);

    double x_add_on = 0;
    for (point = prev_size; point < 20; point++) {
        if (!car_in_front_too_close && ref_vel < speed) {
            ref_vel += 0.112;
        }

        if (car_in_front_too_close && ref_vel > speed) {
            if (speed >= ref_vel - 0.112) {
                ref_vel = speed;
            } else {
                ref_vel -= 0.112;
            }
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

void my_vehicle::updateState(const int prev_size, bool &car_in_front_too_close) {
    car_in_front_too_close = false;

    if (state == STATE_CHANGE_LANE) {
        if (fabs(d - lane2d(target_lane)) < 0.2) {
            state = STATE_IN_LANE;
        }
        car_in_front_too_close = isInLaneAndDirectlyInFrontOfMe(prev_size, target_lane);
    } else if (state == STATE_IN_LANE) {
        // initialize all lane costs to 0
        std::vector<int> lane_cost(NUMBER_OF_LANES, 0);

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
                int own_lane = (i == getLane()) ? -5500 : 0;
                lane_cost[i] += 6000 + own_lane;
            }
        }
        for (unsigned int i = 0; i < lane_cost.size(); i++) {
            if (isInOtherLaneAndDirectlyInBackOfMe(i)) {
                int own_lane = (i == getLane()) ? -3000 : 0;
                lane_cost[i] += 5000 + own_lane;
            }
        }

        for (unsigned int i = 0; i < lane_cost.size(); i++) {
            if (isInLaneAndDirectlyInFrontOfMe(prev_size, i) == true) {
                int own_lane = (i == getLane()) ? -2000 : 0;
                other_vehicle *other = getClosestCarInFrontOfMe(i);
                if (other != nullptr) {
                    lane_cost[i] += abs(3000 + own_lane - fabs(other->s - s) * 10);
                } else {
                    lane_cost[i] += 3000 + own_lane;
                }
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
            if (i != PREFERRED_LANE) {
                lane_cost[i] += 10;
            }
        }

        unsigned int minimal_lane = std::distance(lane_cost.begin(),
                                                  std::min_element(lane_cost.begin(), lane_cost.end()));
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

    }

}

bool my_vehicle::isInSameLaneAndDirectlyInFrontOfMe(const int prev_size) {
    return isInLaneAndDirectlyInFrontOfMe(prev_size, getLane());
}

bool my_vehicle::isInLaneAndDirectlyInFrontOfMe(const int prev_size, const unsigned int lane) {
    bool ret = false;

    for (auto &x : this->otherVehicles.otherVehicles) {
        double other_car_future_s = 0;
        other_vehicle other = x.second;

        other_car_future_s = other.s + prev_size * 0.02 * other.getSpeedMPS();

        if (other.getLane() == lane) {
            if ((other_car_future_s > s) && (other_car_future_s - s < 30)) {
                ret = true;
            }
        }
    }
    return ret;
}

bool my_vehicle::isInOtherLaneAndDirectlyInBackOfMe(const unsigned int lane) {
    bool ret = false;

    for (auto &x : this->otherVehicles.otherVehicles) {
        other_vehicle other = x.second;

        if (other.getLane() == lane) {
            if ((other.s < s && other.s + 120 > s && other.getSpeedMPS() > ref_vel)) {
                ret = true;
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
            if (fabs(other.s - s) < 40) {
                ret = true;
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

other_vehicle *my_vehicle::getClosestCarInFrontOfMe(unsigned int lane) {
    other_vehicle *closest_vehicle_in_front = nullptr;
    for (auto &x : this->otherVehicles.otherVehicles) {
        other_vehicle *other = &x.second;
        if (other->getLane() == lane) {
            if (this->s <= other->s) {
                if (closest_vehicle_in_front == nullptr || closest_vehicle_in_front->s > other->s) {
                    closest_vehicle_in_front = other;
                }
            }
        }

    }
    return closest_vehicle_in_front;
}

double my_vehicle::getTargetSpeed(const int prev_size) {
    other_vehicle *other = nullptr;
    double target_speed_mps = TARGET_SPEED_MPS;
    unsigned int lane = 0;
    switch (state) {
        case STATE_CHANGE_LANE:
            lane = target_lane;
            break;
        case STATE_IN_LANE:
            lane = getLane();
            break;
    }

    bool in_front_of_us = isInLaneAndDirectlyInFrontOfMe(prev_size, lane);
    other = getClosestCarInFrontOfMe(lane);

    if (in_front_of_us) {
        target_speed_mps = fmin(other->getSpeedMPH(), TARGET_SPEED_MPS);
    }
    return target_speed_mps;
}
