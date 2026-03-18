#include "RCL.hpp"
#include <cmath>
#include <vector>

/*
Overview:
This file will contain the RCL class, which will implement the RCL algorithm as follows:
1. Get a sample from each distance sensor
2. Get the estimated position of the robot
3. Raycast from the estimated position, getting what the distance sensors should be reading
4. Compare the raycasted values to the actual distance sensor values, and calculate the error
5. Use that error to update the estimated position of the robot


specifics:
- to raycast, we will use line intersection, to find where the ray and to borders/obstacles intersect
- We will assume that the IMU is correct


things to figure out
-> How to use the error to transform the est. position
	Math for finding sensor pos:
        sp.x=botX+cos(θ)⋅offset
        sp.y=botY+sin(θ)⋅offset
    Then raycasts to all walls, to find the closest wall and uses it.
	    If the sensor is looking at a North/South wall (Y-axis), it calculates: 
            ResultY=WallY−(sin(Angle)⋅val)
        If the sensor is looking at an East/West wall (X-axis), it calculates:
            ResultX=WallX−(cos(Angle)⋅val)
    This gives the robot a "suggested" coordinate. It then subtracts the sensor's 
    offset one last time to find where the center of the robot is.
    Finally it filters if trust is to low:
        - bad angle
        - bad distance reading (too far, or too close)
        - bad intersection (obstacle in the way, or at a corner)


Code layout:
We will have a RCL class, with the following methods:
* Constructor() - Initializes the class, and takes in the 4 distance sensors, the IMU, as well as a vector of coordinates that makes up the obstacles.
* dynamicUpdate() - this will be called every loop, and will perform the RCL algorithm
* staticUpdate(int samples) - A function for getting non-moving distance sensor updates, will take samples number of readings, and will average them to get a more accurate reading
* isValid() - this will check if the reading of the distance sensors is valid
* getPosition() - this will return the RCL adjusted position of the robot

*/

class RCL {
    public:
        RCL(SENSORS sensors, std::vector<std::pair<Coordinate, Coordinate>> obstacles){
            this->sensors = sensors;
            this->obstacles = obstacles;
        }
        SENSORS sensors;
        std::vector<std::pair<Coordinate, Coordinate>> obstacles;
        std::vector<wall> walls = {{{72,72},{72,-72},RIGHT},{{72,-72},{-72,-72},BACK},{{-72,-72},{-72,72},LEFT},{{-72,72},{72,72},FRONT}};
        std::pair<int, double> findWall(Coordinate sensorPos, double Angle, DistanceConfig usage){
            std::pair<int, double> rayResult = {0, 0};
            for (auto& wall : this->walls) {
                double A1 = wall.start.y - wall.end.y;
                double B1 = wall.end.x - wall.start.x;
                double C1 = wall.start.x * wall.end.y - wall.end.x * wall.start.y;

                double A2 = - std::tan(Angle * M_PI / 180.0);
                double B2 = 1;
                double C2 = sensorPos.y - std::tan(Angle * M_PI / 180.0) * sensorPos.x;

                double det = A1 * B2 - A2 * B1;
                if (std::abs(det) < 1e-6) continue; // Lines are parallel, skip

                double x = (B1 * C2 - B2 * C1) / det;
                double y = (C1 * A2 - C2 * A1) / det;
                if (rayResult.second == 0 || std::hypot(x - sensorPos.x, y - sensorPos.y) < rayResult.second) {
                    rayResult = {wall.direction, std::hypot(x - sensorPos.x, y - sensorPos.y)};
                }
            }
            return rayResult;
        }


        Pose dynamicUpdate(Pose pose, DistanceConfig usage) {
            int samples = 1;
            readings avgReadings = {0, 0, 0, 0, 0};
            for (int i = 0; i < samples; i++) {
                if (this->sensors.frontUsing) avgReadings.front += this->sensors.front->get();
                if (this->sensors.backUsing)  avgReadings.back  += this->sensors.back->get();
                if (this->sensors.leftUsing)  avgReadings.left  += this->sensors.left->get();
                if (this->sensors.rightUsing) avgReadings.right += this->sensors.right->get();
                avgReadings.angle += this->sensors.imu->get_heading();
            }

            avgReadings.front /= samples;
            avgReadings.back  /= samples;
            avgReadings.left  /= samples;
            avgReadings.right /= samples;
            avgReadings.angle /= samples;

            avgReadings.angle = std::fmod((avgReadings.angle + 180), 360) - 180;

            /*
            For each sensor, raycast to each wall, and find the closest wall.
            Then for the sensors looking at the EAST - WEST walls, it will calculate the X coord
            and the sensors looking at the NORTH - SOUTH walls will calculate the Y coord.
            
            TODO:
            add filterings for bad readings and obstacles in the way
            add filterings for raycast intersections behind the sensor
            */


            // Front
            Casts casts = {{0,0},{0,0},{0,0},{0,0}};
            double tempAngle = pose.angle;
            Coordinate sensorPos = {
                    pose.pos.x + std::cos(tempAngle * M_PI / 180.0) * usage.Xoffset,
                    pose.pos.y + std::sin(tempAngle * M_PI / 180.0) * usage.Yoffset
            };
            casts.frontSensor = findWall(sensorPos, tempAngle, usage);

            tempAngle = pose.angle + 180;

            Coordinate sensorPos = {
                    pose.pos.x + std::cos(tempAngle * M_PI / 180.0) * usage.Xoffset,
                    pose.pos.y + std::sin(tempAngle * M_PI / 180.0) * usage.Yoffset
                };
            
            casts.backSensor = findWall(sensorPos, tempAngle, usage);


            tempAngle = pose.angle + 90;
            Coordinate sensorPos = {
                    pose.pos.x + std::cos(tempAngle * M_PI / 180.0) * usage.Xoffset,
                    pose.pos.y + std::sin(tempAngle * M_PI / 180.0) * usage.Yoffset
                };

            casts.rightSensor = findWall(sensorPos, tempAngle, usage);

            tempAngle = pose.angle - 90;
            Coordinate sensorPos = {
                    pose.pos.x + std::cos(tempAngle * M_PI / 180.0) * usage.Xoffset,
                    pose.pos.y + std::sin(tempAngle * M_PI / 180.0) * usage.Yoffset
                };
            
            casts.leftSensor = findWall(sensorPos, tempAngle, usage);

            if (casts.frontSensor.second != 0) {
                // Calculate new pose based on front sensor
                if (casts.frontSensor.first == FRONT || casts.frontSensor.first == BACK) {
                    pose.pos.y = (casts.frontSensor.first == FRONT ? 72 : -72) - avgReadings.front * std::sin(pose.angle * M_PI / 180.0);
                    pose.pos.y -= usage.Yoffset * std::sin(pose.angle * M_PI / 180.0);
                } else {
                    pose.pos.x = (casts.frontSensor.first == RIGHT ? 72 : -72) - avgReadings.front * std::cos(pose.angle * M_PI / 180.0);
                    pose.pos.x -= usage.Xoffset * std::cos(pose.angle * M_PI / 180.0);
                }
            }
            if (casts.backSensor.second != 0 && casts.backSensor.second < casts.frontSensor.second) {
                // Calculate new pose based on back sensor
                if (casts.backSensor.first == FRONT || casts.backSensor.first == BACK) {
                    pose.pos.y = (casts.backSensor.first == FRONT ? 72 : -72) - avgReadings.back * std::sin(pose.angle * M_PI / 180.0);
                    pose.pos.y -= usage.Yoffset * std::sin(pose.angle * M_PI / 180.0);
                } else {
                    pose.pos.x = (casts.backSensor.first == RIGHT ? 72 : -72) - avgReadings.back * std::cos(pose.angle * M_PI / 180.0);
                    pose.pos.x -= usage.Xoffset * std::cos(pose.angle * M_PI / 180.0);
                }
            }
            if (casts.rightSensor.second != 0) {
                // Calculate new pose based on right sensor
                if (casts.rightSensor.first == FRONT || casts.rightSensor.first == BACK) {
                    pose.pos.y = (casts.rightSensor.first == FRONT ? 72 : -72) - avgReadings.right * std::sin(pose.angle * M_PI / 180.0);
                    pose.pos.y -= usage.Yoffset * std::sin(pose.angle * M_PI / 180.0);
                } else {
                    pose.pos.x = (casts.rightSensor.first == RIGHT ? 72 : -72) - avgReadings.right * std::cos(pose.angle * M_PI / 180.0);
                    pose.pos.x -= usage.Xoffset * std::cos(pose.angle * M_PI / 180.0);
                }
            }
            if (casts.leftSensor.second != 0 && casts.leftSensor.second < casts.rightSensor.second){
                // Calculate new pose based on left sensor
                if (casts.leftSensor.first == FRONT || casts.leftSensor.first == BACK) {
                    pose.pos.y = (casts.leftSensor.first == FRONT ? 72 : -72) - avgReadings.left * std::sin(pose.angle * M_PI / 180.0);
                    pose.pos.y -= usage.Yoffset * std::sin(pose.angle * M_PI / 180.0);
                } else {
                    pose.pos.x = (casts.leftSensor.first == RIGHT ? 72 : -72) - avgReadings.left * std::cos(pose.angle * M_PI / 180.0);
                    pose.pos.x -= usage.Xoffset * std::cos(pose.angle * M_PI / 180.0);
                }
            }
            return pose;
        }
        Pose staticUpdate(int samples, Pose currentPos, DistanceConfig usage) {
            readings avgReadings = {0, 0, 0, 0};
            Pose newPose = currentPos;
            newPose.angle = 0; 


            for (int i = 0; i < samples; i++) {
                if (usage.xUsing == FRONT || usage.yUsing == FRONT) avgReadings.front += this->sensors.front->get();
                if (usage.xUsing == BACK  || usage.yUsing == BACK)  avgReadings.back  += this->sensors.back->get();
                if (usage.xUsing == LEFT  || usage.yUsing == LEFT)  avgReadings.left  += this->sensors.left->get();
                if (usage.xUsing == RIGHT || usage.yUsing == RIGHT) avgReadings.right += this->sensors.right->get();
                newPose.angle += this->sensors.imu->get_heading();
            }

            avgReadings.front /= samples;
            avgReadings.back  /= samples;
            avgReadings.left  /= samples;
            avgReadings.right /= samples;
            newPose.angle     /= samples;

            // Wrap angle to (-180, 180)
            newPose.angle = std::fmod((newPose.angle + 180), 360) - 180;

   
            auto getTrueDist = [&](double angleOffset, double rawDist) {
                double referenceAngle = std::round(angleOffset / 90) * 90;
                // We use Cosine to get the "flat" distance to the wall
                return std::cos((angleOffset - referenceAngle) * (M_PI / 180.0)) * rawDist;
            };


            if (usage.xUsing == FRONT)      newPose.pos.x = getTrueDist(newPose.angle, avgReadings.front);
            else if (usage.xUsing == BACK)  newPose.pos.x = getTrueDist(newPose.angle + 180, avgReadings.back);
            else if (usage.xUsing == RIGHT) newPose.pos.x = getTrueDist(newPose.angle + 90, avgReadings.right);
            else if (usage.xUsing == LEFT)  newPose.pos.x = getTrueDist(newPose.angle - 90, avgReadings.left);

            if (usage.yUsing == FRONT)      newPose.pos.y = getTrueDist(newPose.angle, avgReadings.front);
            else if (usage.yUsing == BACK)  newPose.pos.y = getTrueDist(newPose.angle + 180, avgReadings.back);
            else if (usage.yUsing == RIGHT) newPose.pos.y = getTrueDist(newPose.angle + 90, avgReadings.right);
            else if (usage.yUsing == LEFT)  newPose.pos.y = getTrueDist(newPose.angle - 90, avgReadings.left);
            //Now we have the distance from the wall, we need to apply the offset, and convert to inches, and apply + or -
            if(usage.xUsing != NONE){
                newPose.pos.x += usage.Xoffset;
                newPose.pos.x *= 0.0393701; // Convert from mm to inches
            }
            if(usage.yUsing != NONE){
                newPose.pos.y += usage.Yoffset;
                newPose.pos.y *= 0.0393701; // Convert from mm to inches
            }

            Pose AdjustedPose = newPose;
            if (usage.yUsing != NONE){
                if (std::fmod(((std::round(((newPose.angle)+usage.yUsing)/90)) + 180), 360) - 180 == static_cast<int>(FRONT)){
                AdjustedPose.pos.y = -newPose.pos.y;
                }
            }
            if(usage.xUsing != NONE){
                if (std::fmod(((std::round(((newPose.angle)+usage.xUsing)/90)) + 180), 360) - 180 == static_cast<int>(RIGHT)){
                    AdjustedPose.pos.x = -newPose.pos.x;
                }
            }
            return AdjustedPose;
        }
};