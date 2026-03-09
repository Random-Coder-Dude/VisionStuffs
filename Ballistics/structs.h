#pragma once

#include <stdio.h>

/**
 * @file structs.h
 * @brief Core data structures and vector math utilities for the ballistics simulator.
 *
 * Provides Vec3 (3D vector), ChassisSpeeds (robot drivetrain velocity), and a
 * full set of vector arithmetic helpers used throughout the physics pipeline.
 */

/* =========================================================================
 * Data Structures
 * ========================================================================= */

/**
 * @brief A 3-dimensional vector used for positions, velocities, forces, and spin.
 *
 * Coordinate convention (field-relative, robot shooter looking +X):
 *   x — forward/backward (meters)
 *   y — left/right       (meters)
 *   z — up/down          (meters, positive = up)
 */
typedef struct {
    double x; /**< X component (forward, meters) */
    double y; /**< Y component (lateral, meters) */
    double z; /**< Z component (vertical, meters) */
} Vec3;

/**
 * @brief Robot drivetrain velocity in the field frame.
 *
 * Used to apply a moving-platform correction to the initial ball velocity so
 * the simulator accounts for the robot moving while shooting.
 *
 *   vx    — forward velocity  (m/s)
 *   vy    — lateral velocity  (m/s, positive = left)
 *   omega — yaw rate          (rad/s, positive = CCW when viewed from above)
 */
typedef struct {
    double vx;    /**< Forward velocity  (m/s) */
    double vy;    /**< Lateral velocity  (m/s) */
    double omega; /**< Yaw rate          (rad/s) */
} ChassisSpeeds;

/* =========================================================================
 * Constructor Helpers
 * ========================================================================= */

/**
 * @brief Construct a ChassisSpeeds value.
 * @param vx    Forward velocity  (m/s)
 * @param vy    Lateral velocity  (m/s)
 * @param omega Yaw rate          (rad/s)
 * @return Initialized ChassisSpeeds struct.
 */
ChassisSpeeds createChassisSpeeds(double vx, double vy, double omega);

/**
 * @brief Construct a Vec3 value.
 * @param x X component
 * @param y Y component
 * @param z Z component
 * @return Initialized Vec3 struct.
 */
Vec3 createVec3(double x, double y, double z);

/* =========================================================================
 * Vector Arithmetic
 * ========================================================================= */

/**
 * @brief Scale a vector by a scalar: result = scalar * a.
 * @param scalar Multiplicative scale factor.
 * @param a      Input vector.
 * @return Scaled vector.
 */
Vec3 scalarMultVec3(double scalar, Vec3 a);

/**
 * @brief Add two vectors component-wise: result = a + b.
 * @param a First operand.
 * @param b Second operand.
 * @return Sum vector.
 */
Vec3 addVec3(Vec3 a, Vec3 b);

/**
 * @brief Compute the Euclidean magnitude (L2 norm) of a vector.
 * @param a Input vector.
 * @return Non-negative scalar magnitude.
 */
double magnitudeVec3(Vec3 a);

/**
 * @brief Return a unit-length copy of the vector.
 *
 * If the magnitude is zero (or below epsilon), the zero vector is returned
 * to avoid division-by-zero.
 *
 * @param a Input vector.
 * @return Unit vector in the same direction, or zero vector if |a| ≈ 0.
 */
Vec3 normalizeVec3(Vec3 a);

/**
 * @brief Compute the cross product: result = a × b.
 *
 * The right-hand rule applies. Used by the Magnus-effect calculation to find
 * the force direction perpendicular to both spin axis and velocity.
 *
 * @param a First vector.
 * @param b Second vector.
 * @return Cross product vector.
 */
Vec3 crossVec3(Vec3 a, Vec3 b);

/**
 * @brief Linearly interpolate between two vectors.
 *
 * result = a + t * (b - a)
 *
 * @param a Start vector (t = 0).
 * @param b End   vector (t = 1).
 * @param t Interpolation parameter in [0, 1].
 * @return Interpolated vector.
 */
Vec3 lerpVec3(Vec3 a, Vec3 b, double t);

/**
 * @brief Print a Vec3 to stdout for debugging.
 * @param a Vector to print.
 */
void printVec3(Vec3 a);
