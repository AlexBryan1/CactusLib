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

struct DirectionsXY{
    int x;
    int y;
};

enum direction{
    FRONT,
    BACK,
    LEFT,
    RIGHT
};