//
// Created by Ralf on 5/5/2018.
//
#include <vector>
#include "other_vehicles.h"
#include "other_vehicle.h"


#define LOGURU_WITH_STREAMS 1
#include "loguru/loguru.hpp"

void other_vehicles::update(const nlohmann::json &sensor_fusion) {
    std::vector<unsigned int> found_cars;
    for (unsigned int i = 0; i < sensor_fusion.size(); i++) {
        unsigned int id = sensor_fusion[i][0];
        double x, y, s, d, vx, vy;
        x = sensor_fusion[i][1];
        y = sensor_fusion[i][2];
        vx = sensor_fusion[i][3];
        vy = sensor_fusion[i][4];
        s = sensor_fusion[i][5];
        d = sensor_fusion[i][6];

        found_cars.push_back(id);
        auto pos = this->otherVehicles.find( id );

        if( pos == this->otherVehicles.end() ){
            LOG_S(INFO) << "New approaching car: " << id;
            this->otherVehicles.insert(std::make_pair<unsigned int&, other_vehicle>(id, other_vehicle(id)));
            pos = this->otherVehicles.find( id );
        }
        pos->second.update(x,y,s,d,vx,vy);
    }

    for (auto it = this->otherVehicles.cbegin(); it != this->otherVehicles.cend() /* not hoisted */; /* no increment */)
    {
        if (std::find(found_cars.begin(), found_cars.end(), it->first) == found_cars.end())
        {
            LOG_S(INFO) << "Car not seen anymore: " << it->first;
            this->otherVehicles.erase(it++);    // or "it = m.erase(it)" since C++11
        }
        else
        {
            ++it;
        }
    }
}

other_vehicles::other_vehicles() {

}
