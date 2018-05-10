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


