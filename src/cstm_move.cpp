#include "cstm_move.hpp"
#include <cmath>
#include <algorithm>

// ─── Runtime state (populated by cstm_move_init) ─────────────────────────────
static pros::MotorGroup* g_leftMotors  = nullptr;
static pros::MotorGroup* g_rightMotors = nullptr;
static double            g_wheelDiam   = 4.0;   // inches — overwritten by init
static double            g_gearRatio   = 1.667; // motor:wheel — overwritten by init
// ─────────────────────────────────────────────────────────────────────────────

// ─── Wall-Tracking PD Gains ───────────────────────────────────────────────────
// Units: WALL_KP is in (mV / mm of distance error).
//        At KP=30, a 10 mm wall error produces a 300 mV steering correction.
//
// Tuning procedure:
//   1. Set WALL_KD = 0.  Raise WALL_KP until the robot corrects to the target
//      distance without side-to-side oscillation.
//   2. If the robot oscillates, raise WALL_KD in small steps to dampen it.
//   3. Once stable, raise WALL_KP further if correction still feels sluggish.
static constexpr double WALL_KP = 30.0; // TODO: tune for your robot
static constexpr double WALL_KD =  5.0; // TODO: tune for your robot

// pros::Distance::get() returns PROS_ERR (~UINT32_MAX) when no object is in range.
// Readings above this threshold (mm) are treated as invalid → zero correction.
static constexpr double SENSOR_MAX_VALID_MM = 2000.0;
// ─────────────────────────────────────────────────────────────────────────────

// Helper: maps a pros::v5::MotorGears enum to its cartridge's free-spin RPM.
// Used to compute the gear ratio between the motor shaft and the driven wheel.
// pros::v5::MotorGears is the PROS 4 scoped enum returned by get_gearing().
static double cartridgeRPM(pros::v5::MotorGears gearset) {
    switch (gearset) {
        case pros::v5::MotorGears::red:     return 100.0; // 100 RPM cartridge
        case pros::v5::MotorGears::green:   return 200.0; // 200 RPM cartridge
        case pros::v5::MotorGears::blue:    return 600.0; // 600 RPM cartridge (your robot)
        case pros::v5::MotorGears::invalid: return 600.0; // fallback: assume blue
        default:                            return 600.0;
    }
}

void cstm_move_init(lemlib::Drivetrain& dt) {
    // Extract motor group pointers directly from the LemLib drivetrain struct.
    // These are the same MotorGroup objects passed to lemlib::Chassis — no duplication.
    g_leftMotors  = dt.leftMotors;
    g_rightMotors = dt.rightMotors;

    // Extract wheel diameter in inches from the LemLib drivetrain config.
    // For lemlib::Omniwheel::NEW_4 this will be 4.0 inches.
    g_wheelDiam = dt.wheelDiameter;

    // Derive the gear ratio (motor shaft RPM : wheel RPM) from the drivetrain.
    // LemLib's drivetrain.rpm is the *output* RPM at the wheel.
    // pros::MotorGroup::get_position() returns *motor shaft* degrees.
    // To convert shaft degrees → inches of travel, we need to know how many
    // motor shaft revolutions equal one wheel revolution.
    //
    // gearRatio = cartridge_RPM / wheel_output_RPM
    //           = 600           / 360             = 1.667  (for your blue + 360 rpm config)
    //
    // If the motor group returns an empty gearset vector, fall back to blue (600 RPM).
    // pros::MotorGroup::get_gearing() returns a single pros::v5::MotorGears enum
    // value in PROS 4 — not a vector. Pass it directly to cartridgeRPM().
    double motorRPM = cartridgeRPM(g_leftMotors->get_gearing());

    g_gearRatio = motorRPM / static_cast<double>(dt.rpm);
}

void driveWallTrack(pros::Distance& distSensor,
                    double targetDistIn,
                    double driveDistIn,
                    WallSide wallSide,
                    int    baseVoltage,
                    int    timeout) {

    // Safety guard — do nothing if cstm_move_init() was never called.
    if (g_leftMotors == nullptr || g_rightMotors == nullptr) return;

    // Convert target wall distance from inches to millimeters (sensor speaks mm).
    const double targetDistMM = targetDistIn * 25.4;

    // Convert drive distance from inches to motor shaft degrees for progress tracking.
    // Formula: (distance / wheel_circumference) * 360 * gearRatio = shaft degrees
    const double driveDistDeg = (driveDistIn / (M_PI * g_wheelDiam)) * 360.0 * g_gearRatio;

    // Zero both encoder positions so distance tracking starts from 0 at call time.
    g_leftMotors->tare_position();
    g_rightMotors->tare_position();

    double prevWallError     = 0.0;
    const uint32_t startTime = pros::millis();

    while (pros::millis() - startTime < static_cast<uint32_t>(timeout)) {

        // ── Forward distance check ────────────────────────────────────────────
        // Average the absolute shaft encoder degrees of both groups and compare
        // to the target degrees derived from driveDistIn above.
        double traveledDeg = (std::abs(g_leftMotors->get_position()) +
                              std::abs(g_rightMotors->get_position())) / 2.0;

        if (traveledDeg >= driveDistDeg) break;

        // ── Wall distance error ───────────────────────────────────────────────
        // pros::Distance::get() returns mm, or PROS_ERR when out of range.
        // Guard against invalid readings to prevent runaway corrections.
        double sensorMM  = static_cast<double>(distSensor.get());
        double wallError = 0.0;

        if (sensorMM < SENSOR_MAX_VALID_MM) {
            // positive → too far from wall → steer toward wall
            // negative → too close to wall → steer away from wall
            wallError = targetDistMM - sensorMM;
        }

        double derivative = wallError - prevWallError;
        prevWallError = wallError;

        int correction = static_cast<int>(WALL_KP * wallError + WALL_KD * derivative);
        correction = std::clamp(correction, -baseVoltage, baseVoltage);

        // ── Differential voltage steering ────────────────────────────────────
        // Correction voltage is added to one side and subtracted from the other.
        //
        // WallSide::RIGHT — sensor on right side:
        //   wallError > 0 (too far)  → boost left, reduce right → turns right toward wall
        //   wallError < 0 (too close) → reduce left, boost right → turns left away from wall
        //
        // WallSide::LEFT — sensor on left side:
        //   wallError > 0 (too far)  → boost right, reduce left → turns left toward wall
        //   wallError < 0 (too close) → reduce right, boost left → turns right away from wall
        int leftVoltage, rightVoltage;

        if (wallSide == WallSide::RIGHT) {
            leftVoltage  = baseVoltage + correction;
            rightVoltage = baseVoltage - correction;
        } else {
            leftVoltage  = baseVoltage - correction;
            rightVoltage = baseVoltage + correction;
        }

        // Clamp final voltages to valid range. Floor at 0 to prevent reversing
        // when a large correction would otherwise drive a side backward.
        leftVoltage  = std::clamp(leftVoltage,  0, 12000);
        rightVoltage = std::clamp(rightVoltage, 0, 12000);

        g_leftMotors->move_voltage(leftVoltage);
        g_rightMotors->move_voltage(rightVoltage);

        pros::delay(10); // 10 ms loop — required to yield to the PROS scheduler
    }

    g_leftMotors->move_voltage(0);
    g_rightMotors->move_voltage(0);
}
