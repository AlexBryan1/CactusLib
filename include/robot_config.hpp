#pragma once

// robot_config.hpp
// Extern declarations for drivetrain objects defined in main.cpp.
// Include this header in any file that needs to access the motor groups or IMU.

#include "main.h"

// Left motor group — ports 3 (reversed), 4, 5 (reversed) — defined in main.cpp
extern pros::MotorGroup leftMotors;

// Right motor group — ports 6, 7, 9 (reversed) — defined in main.cpp
extern pros::MotorGroup rightMotors;

// Inertial sensor — port 10 — defined in main.cpp
extern pros::Imu imu;
