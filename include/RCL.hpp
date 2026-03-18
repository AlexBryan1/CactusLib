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
    double angle;
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


struct DistanceConfig{
    int xUsing;
    int yUsing;
    float Xoffset;
    float Yoffset;
};

struct wall{
    Coordinate start;
    Coordinate end;
    int direction;
};

struct Casts{
    std::pair<int, double> frontSensor;
    std::pair<int, double> backSensor;
    std::pair<int, double> leftSensor;
    std::pair<int, double> rightSensor;
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