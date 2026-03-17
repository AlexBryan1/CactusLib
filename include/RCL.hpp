#include "main.h"
#include <vector>


struct Coordinate {
    int x;
    int y;
};

struct Pose {
    Coordinate pos;
    double angle;
};

struct readings {
    double front;
    double back;
    double left;
    double right;
};

struct SENSORS{
    bool frontUsing;
    bool backUsing;
    bool leftUsing;
    bool rightUsing;
    pros::Distance* front;
    pros::Distance* back;
    pros::Distance* left;
    pros::Distance* right;
    pros::IMU* imu;
};

struct usage{
    bool frontUsing;
    bool backUsing;
    bool leftUsing;
    bool rightUsing;
};

struct DistanceConfig{
    int x;
    int y;
    float Xoffset;
    float Yoffset;
};

enum direction{
    FRONT,
    RIGHT,
    BACK,
    LEFT,
    NONE
};


class RCL {
    public:
        RCL(SENSORS sensors, std::vector<std::pair<Coordinate, Coordinate>> obstacles);
        SENSORS sensors;
        Pose staticUpdate(int samples, Pose currentPos, DistanceConfig usage);
        Pose dynamicUpdate(Pose pose);
    };