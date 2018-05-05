//
// Created by Ralf on 5/5/2018.
//

#ifndef PATH_PLANNING_OTHER_VEHICLES_H
#include "json/json.hpp"
#include "other_vehicle.h"
#include <map>
#define PATH_PLANNING_OTHER_VEHICLES_H


class other_vehicles {
private:
    std::map<unsigned int, other_vehicle> otherVehicles;
public:
    other_vehicles();
    void update(const nlohmann::json& sensor_fusion);
};


#endif //PATH_PLANNING_OTHER_VEHICLES_H
