#pragma once

/**
 * @file initialVelo.h
 * @brief Converts shooter settings (RPM, hood angle, turret angle) into an
 *        initial 3-D ball velocity vector, including robot-motion compensation.
 *
 * Two look-up tables (LUTs) with linear interpolation map:
 *   RPM        → exit speed  (m/s)
 *   Hood angle → launch pitch (degrees)
 *
 * The final velocity accounts for the robot's translational and rotational
 * motion at the moment of release so the projectile paths are field-relative.
 *
 * Additionally, the angular velocity (spin) of the ball imparted by the
 * shooter flywheel is computed here and returned via an out-parameter so the
 * physics integrator can apply the Magnus effect correctly.
 */

#include "structs.h"

/**
 * @brief Compute the initial ball velocity vector and spin from shooter params.
 *
 * Steps performed:
 *   1. Look up exit speed from RPM via linear-interpolated LUT.
 *   2. Look up launch pitch from hood angle via linear-interpolated LUT.
 *   3. Convert turret yaw + pitch to a unit direction vector.
 *   4. Scale by exit speed to get shooter-frame velocity.
 *   5. Add robot translational velocity (vx, vy) in the field frame.
 *   6. Add velocity contribution from robot rotation (omega × offset).
 *   7. Compute ball spin angular velocity from flywheel surface speed.
 *
 * @param RPM          Flywheel speed (rev/min). Clamped to LUT bounds.
 * @param hoodAngle    Hood mechanism angle (degrees). Clamped to LUT bounds.
 * @param turretAngle  Turret yaw angle (degrees, 0 = forward).
 * @param robot        Robot drivetrain velocity at the moment of release.
 * @param[out] spinOut Angular velocity vector of the ball (rad/s) after
 *                     the flywheel imparts spin. Pass NULL to ignore.
 * @return Initial ball velocity in the field frame (m/s).
 */
Vec3 calculateInitialVelocity(double RPM, double hoodAngle, double turretAngle,
                              ChassisSpeeds robot, Vec3 *spinOut);
