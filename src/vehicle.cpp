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
#include "vehicle.h"

#define LOGURU_WITH_STREAMS 1

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
}

unsigned int vehicle::getLane() {
    return round((this->d - 2) / 4);
}

int vehicle::lane2d(int lane) {
    return lane * 4 + 2;
}