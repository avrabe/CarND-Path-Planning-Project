//
// Created by Ralf on 5/4/2018.
//

#ifndef PATH_PLANNING_VEHICLE_H
#define PATH_PLANNING_VEHICLE_H


class vehicle {
private:
    unsigned int id;
    double x, y, vx, vy, s, d;
public:
    explicit vehicle(unsigned int my_id);

    vehicle();

    void update(double x, double y, double vx, double vy, double s, double d);
};


#endif //PATH_PLANNING_VEHICLE_H
