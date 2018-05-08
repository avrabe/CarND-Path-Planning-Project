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

    if (car_in_front_too_close) {
        LOG_S(INFO) << "too close";
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
        if (!car_in_front_too_close && ref_vel < TARGET_SPEED_MPS) {
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
            car_in_front_too_close = isInSameLaneAndDirectlyInFrontOfMe(prev_size);
            break;
        case STATE_IN_LANE:
            // initialize all lane costs to 0
            car_in_front_too_close = isInSameLaneAndDirectlyInFrontOfMe(prev_size);
            std::vector<int> lane_cost(NUM_LANES, 0);


            switch (getLane()) { // Don't switch two lanes at a time
                case 0:
                    lane_cost[2] = 1000;
                    break;
                case 2:
                    lane_cost[0] = 1000;
                    break;
            }

            // Prefer lanes with no car
            for (unsigned int i = 0; i < lane_cost.size(); i++) {
                if (isLaneFree(prev_size) == false) {
                    lane_cost[i] += 500;
                }
            }

            std::vector<int> cars_in_front_of_lane_too_close(NUM_LANES, false);
            for (unsigned int i = 0; i < lane_cost.size(); i++) {
                if (isInSameLaneAndDirectlyInFrontOfMe(prev_size, i) == true) {
                    lane_cost[i] += 250;
                }
            }

            // Prefer the preferred lane
            for (unsigned int i = 0; i < lane_cost.size(); i++) {
                if (getLane() != PREFERED_LANE) {
                    lane_cost[i] += 10;
                }
            }

            int minimal_lane = std::distance(lane_cost.begin(), std::min_element(lane_cost.begin(), lane_cost.end()));
            if (minimal_lane != getLane()) {
                state = STATE_CHANGE_LANE;
                target_lane = minimal_lane;
                car_in_front_too_close = false;
                LOG_S(INFO) << "Switching to lane " << minimal_lane;
            }
            /*
            for (auto &x : otherVehicles.otherVehicles) {
                other_vehicle other = x.second;
                double other_car_future_s = other.s + prev_size * 0.02 * other.getSpeedMPS();

                if (other.getLane() == getLane()) {
                    if ((other_car_future_s > s) &&
                        (other_car_future_s - s < 30)) {
                    lane_cost[other.getLane()] += 1;
                    car_in_front_too_close = true;
                    cars_in_front_of_lane_too_close[getLane()] = true;
                    } // else lane seems free
                } else {
                    if (fabs(other.s - s) < 30) { // car is on another lane and blocking us
                        lane_cost[other.getLane()] += 1;
                        cars_in_front_of_lane_too_close[other.getLane()] = true;
                    } else if  (other.s < s && other.s + 60 > s &&
                                other.getSpeedMPS() > ref_vel) { // car is in another lane and about to overtake us
                        cars_in_front_of_lane_too_close[other.getLane()] = false;
                        lane_cost[other.getLane()] += 1000;
                    }
                }

                state = STATE_CHANGE_LANE;

                if (getLane() >= 1 && lane_cost[getLane() - 1] < lane_cost[getLane()]) {
                    target_lane = getLane() - 1;
                }
                else if (getLane() < NUM_LANES - 1 && lane_cost[getLane() + 1] < lane_cost[getLane()]) {
                    target_lane = getLane() + 1;
                }
                else if (getLane() != PREFERED_LANE && lane_cost[PREFERED_LANE] == 0) {
                    target_lane = PREFERED_LANE;
                } else {
                    state = STATE_IN_LANE;
                };
                if (state == STATE_CHANGE_LANE) {
                    LOG_S(INFO) << "changing lane to " << target_lane;
                    //car_in_front_too_close = car_in_lane_too_close[getLane()];
                }
             */
            break;
    }
}

bool my_vehicle::isInSameLaneAndDirectlyInFrontOfMe(const int prev_size) {
    return isInSameLaneAndDirectlyInFrontOfMe(prev_size, getLane());
}

bool my_vehicle::isInSameLaneAndDirectlyInFrontOfMe(const int prev_size, const unsigned int lane) {
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

