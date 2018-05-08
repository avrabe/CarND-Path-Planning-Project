//
// Created by Ralf on 5/4/2018.
//

#ifndef PATH_PLANNING_VEHICLE_H
#define PATH_PLANNING_VEHICLE_H


class vehicle {
private:
    double previous_x, previous_y, previous_s, previous_d;

protected:
    void update(double x, double y, double s, double d);

    int lane2d(int lane);

public:
    unsigned int id;
    double x, y;
    double s, d;
    explicit vehicle(unsigned int my_id);
    vehicle();

    bool isInSameLane(vehicle &other);

    int getLane();

};


#endif //PATH_PLANNING_VEHICLE_H
