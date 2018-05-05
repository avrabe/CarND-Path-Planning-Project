//
// Created by Ralf on 5/4/2018.
//

#ifndef PATH_PLANNING_VEHICLE_H
#define PATH_PLANNING_VEHICLE_H


class vehicle {
private:
    double x, y, d;
    double previous_x, previous_y, previous_s, previous_d;

protected:
    void update(double x, double y, double s, double d);

    unsigned int getLane();
public:
    unsigned int id;
    double s;
    explicit vehicle(unsigned int my_id);
    vehicle();

    bool isInSameLane(vehicle &other);

};


#endif //PATH_PLANNING_VEHICLE_H
