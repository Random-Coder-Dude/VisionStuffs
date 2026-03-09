#pragma once

/**
 * @file forwardPass.h
 * @brief Numerical trajectory integrator using RK4 (Runge-Kutta 4th order).
 *
 * Simulates the flight of a ball from shooter exit to the target height,
 * accounting for gravity, aerodynamic drag, and Magnus (gyroscopic) lift.
 *
 * Integration is split into two phases:
 *   Phase 1 (ascent)  — from launch until the apex (vz flips negative).
 *   Phase 2 (descent) — from apex until the ball descends through goalZ.
 *
 * RK4 is used instead of simple Euler integration because it provides
 * 4th-order accuracy (error ~ dt^4) with the same step count, significantly
 * reducing position drift on longer trajectories where Magnus forces are large.
 */

#include <stdbool.h>
#include "structs.h"

/**
 * @brief Results returned from a single trajectory simulation.
 */
typedef struct {
    Vec3   finalPosition; /**< XYZ position where the ball crossed goalZ (m).
                               Only meaningful when scoreable = true.          */
    double shotTime;      /**< Total flight time from launch to goalZ (s).
                               Only meaningful when scoreable = true.          */
    double maxHeight;     /**< Maximum altitude reached during flight (m).
                               Always set, even for invalid trajectories.      */
    bool   scoreable;     /**< True IFF the ball crossed goalZ while descending.
                               This is the single validity gate used by the
                               scorer — no separate valid/comingFromTop split.
                               False means the ball never reached goalZ, or
                               it crossed goalZ on the way UP (ascent only),
                               or it hit the floor before reaching goalZ.      */
    double RPM;           /**< Flywheel RPM used for this simulation.          */
    double hoodAngle;     /**< Hood angle  (deg) used for this simulation.     */
    double turretAngle;   /**< Turret angle (deg) used for this simulation.    */
    Vec3   spin;          /**< Ball angular velocity at launch (rad/s).        */
} SimResult;

/**
 * @brief Simulate ball flight from launch to target height using RK4 integration.
 *
 * The integrator advances the 6-DOF state [position, velocity] forward in time
 * with a fixed timestep of dt = 0.001 s. RK4 evaluates the force model four
 * times per step for a much more accurate result than single-step (Euler) methods,
 * especially important when Magnus forces are significant.
 *
 * scoreable is set to true only when ALL of these conditions hold:
 *   1. The ball's apex was at or above goalZ  (it was high enough).
 *   2. The ball crossed goalZ while vz < 0    (it was on the way down).
 *   3. The crossing happened before z hit 0   (it didn't land short first).
 *
 * Linear interpolation is used for sub-step precision at the goalZ crossing.
 *
 * @param rpm           Flywheel speed (rev/min).
 * @param hoodAngle     Hood mechanism angle (degrees).
 * @param turretAngle   Turret yaw angle (degrees; 0 = straight ahead).
 * @param goalZ         Target height to intersect on descent (meters above floor).
 * @param robotVelocity Robot drivetrain velocity at the moment of launch.
 * @return SimResult with finalPosition, shotTime, maxHeight, and scoreable flag.
 */
SimResult calculateTrajectory(double rpm, double hoodAngle, double turretAngle,
                              double goalZ, ChassisSpeeds robotVelocity);
