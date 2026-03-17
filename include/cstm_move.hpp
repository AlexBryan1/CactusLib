#pragma once


#include "main.h"        // PROS umbrella — provides all pros:: types
#include "lemlib/api.hpp" // provides lemlib::Drivetrain, lemlib::Chassis, etc.

// WallSide — which side of the robot the distance sensor is mounted on.
//   RIGHT: too close to wall → steer left  (reduce right voltage, boost left)
//   LEFT:  too close to wall → steer right (reduce left voltage,  boost right)
enum class WallSide { LEFT, RIGHT };


// Extracts leftMotors, rightMotors, wheelDiameter, and gear ratio directly
// from the provided lemlib::Drivetrain so no values need to be re-entered.
// Must be called once in initialize() before any driveWallTrack() calls.
//
//   dt : the same lemlib::Drivetrain instance passed to lemlib::Chassis
void cstm_move_init(lemlib::Drivetrain& dt);

// driveWallTrack — drives forward a fixed distance while maintaining a target
// distance from a wall using a lateral distance sensor and a PD steering loop.
//
// Forward progress is tracked via motor encoder degrees (averaged across both
// motor groups) and converted to inches using the wheel diameter and gear ratio
// that were registered in cstm_move_init().
//
// The distance sensor applies a differential voltage correction each 10ms cycle:
//   - Robot too far from wall  → positive correction → steers toward wall
//   - Robot too close to wall  → negative correction → steers away from wall
//
// cstm_move_init() must be called before this function is used.
//
// Parameters:
//   distSensor   : pros::Distance sensor mounted on the wall-facing side
//   targetDistIn : desired distance to maintain from the wall, in inches
//   driveDistIn  : total forward distance to travel, in inches
//   wallSide     : side of the robot the sensor is on (default: WallSide::RIGHT)
//   baseVoltage  : base forward drive voltage, 0–12000 mV  (default: 7000)
//   timeout      : max run duration in milliseconds        (default: 5000)
void driveWallTrack(pros::Distance& distSensor,
                    double targetDistIn,
                    double driveDistIn,
                    WallSide wallSide   = WallSide::RIGHT,
                    int    baseVoltage  = 7000,
                    int    timeout      = 5000);
