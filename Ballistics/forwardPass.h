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
    Vec3   finalPosition; /**< XYZ position where the ball crosses goalZ (m). */
    double shotTime;      /**< Total flight time from launch to goalZ (s).     */
    double maxHeight;     /**< Maximum altitude reached during flight (m).     */
    bool   comingFromTop; /**< True if the ball descends through goalZ (good). */
    bool   valid;         /**< True if the ball ever reached goalZ height.     */
    double RPM;           /**< Flywheel RPM used for this simulation.          */
    double hoodAngle;     /**< Hood angle (deg) used for this simulation.      */
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
 * Phase transitions:
 *   - Ascent ends when vz < 0 (ball has passed the apex).
 *   - Descent ends when position.z <= goalZ (ball has crossed the target plane).
 *   - Linear interpolation is used for sub-step precision at the crossing.
 *
 * @param rpm          Flywheel speed (rev/min).
 * @param hoodAngle    Hood mechanism angle (degrees).
 * @param turretAngle  Turret yaw angle (degrees; 0 = straight ahead).
 * @param goalZ        Target height to intersect on descent (meters above floor).
 * @param robotVelocity Robot drivetrain velocity at the moment of launch.
 * @return SimResult containing final position, flight time, max height, and flags.
 */
SimResult calculateTrajectory(double rpm, double hoodAngle, double turretAngle,
                              double goalZ, ChassisSpeeds robotVelocity);
