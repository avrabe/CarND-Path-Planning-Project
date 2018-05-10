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
#include "other_vehicles.h"


#define LOGURU_WITH_STREAMS 1
#include "loguru/loguru.hpp"

void other_vehicles::update(const nlohmann::json &sensor_fusion) {
    std::vector<unsigned int> found_cars;
    for (unsigned int i = 0; i < sensor_fusion.size(); i++) {
        unsigned int id = sensor_fusion[i][0];
        double x;
        double y;
        double s;
        double d;
        double vx;
        double vy;
        x = sensor_fusion[i][1];
        y = sensor_fusion[i][2];
        vx = sensor_fusion[i][3];
        vy = sensor_fusion[i][4];
        s = sensor_fusion[i][5];
        d = sensor_fusion[i][6];

        found_cars.push_back(id);
        auto pos = this->otherVehicles.find( id );

        if( pos == this->otherVehicles.end() ){
            this->otherVehicles.insert(std::make_pair<unsigned int&, other_vehicle>(id, other_vehicle(id)));
            pos = this->otherVehicles.find( id );
        }
        pos->second.update(x,y,s,d,vx,vy);
    }

    for (auto it = this->otherVehicles.cbegin(); it != this->otherVehicles.cend() /* not hoisted */; /* no increment */)
    {
        if (std::find(found_cars.begin(), found_cars.end(), it->first) == found_cars.end())
        {
            this->otherVehicles.erase(it);    // or "it = m.erase(it)" since C++11
            it++;
        }
        else
        {
            ++it;
        }
    }
}

other_vehicles::other_vehicles() {

}
