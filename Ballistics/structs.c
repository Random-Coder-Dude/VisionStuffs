/**
 * @file structs.c
 * @brief Implementations of Vec3 and ChassisSpeeds constructors and vector math.
 *
 * All operations work on doubles for physics-simulation precision.
 * No heap allocation is used; every function returns a value-type struct.
 */

#include <math.h>
#include "structs.h"

/** Threshold below which a vector magnitude is treated as zero. */
#define VEC3_EPSILON 1e-12

/* =========================================================================
 * Constructors
 * ========================================================================= */

/**
 * @brief Construct a ChassisSpeeds value.
 */
ChassisSpeeds createChassisSpeeds(double vx, double vy, double omega) {
    ChassisSpeeds result;
    result.vx    = vx;
    result.vy    = vy;
    result.omega = omega;
    return result;
}

/**
 * @brief Construct a Vec3 value.
 */
Vec3 createVec3(double x, double y, double z) {
    Vec3 result;
    result.x = x;
    result.y = y;
    result.z = z;
    return result;
}

/* =========================================================================
 * Arithmetic
 * ========================================================================= */

/**
 * @brief Scale a vector: result = scalar * a.
 */
Vec3 scalarMultVec3(double scalar, Vec3 a) {
    return createVec3(scalar * a.x, scalar * a.y, scalar * a.z);
}

/**
 * @brief Add two vectors: result = a + b.
 */
Vec3 addVec3(Vec3 a, Vec3 b) {
    return createVec3(a.x + b.x, a.y + b.y, a.z + b.z);
}

/**
 * @brief Euclidean magnitude of a vector.
 */
double magnitudeVec3(Vec3 a) {
    return sqrt(a.x * a.x + a.y * a.y + a.z * a.z);
}

/**
 * @brief Return a unit-length copy of the vector.
 *
 * Guards against division-by-zero using VEC3_EPSILON.
 */
Vec3 normalizeVec3(Vec3 a) {
    double magnitude = magnitudeVec3(a);
    if (magnitude < VEC3_EPSILON) return createVec3(0.0, 0.0, 0.0);
    return scalarMultVec3(1.0 / magnitude, a);
}

/**
 * @brief Cross product: result = a × b.
 *
 * Computed via the standard determinant expansion:
 *   | i   j   k  |
 *   | ax  ay  az |
 *   | bx  by  bz |
 */
Vec3 crossVec3(Vec3 a, Vec3 b) {
    return createVec3(
        a.y * b.z - a.z * b.y,
        a.z * b.x - a.x * b.z,
        a.x * b.y - a.y * b.x
    );
}

/**
 * @brief Linear interpolation: result = a + t*(b-a).
 *
 * Used during trajectory integration to pinpoint the exact moment the ball
 * crosses a target height between two simulation steps.
 */
Vec3 lerpVec3(Vec3 a, Vec3 b, double t) {
    /* Equivalent to: a*(1-t) + b*t, written to avoid catastrophic cancellation. */
    return addVec3(a, scalarMultVec3(t, addVec3(b, scalarMultVec3(-1.0, a))));
}

/**
 * @brief Print a Vec3 to stdout for debugging.
 */
void printVec3(Vec3 a) {
    printf("Vec3: x=%.4f, y=%.4f, z=%.4f\n", a.x, a.y, a.z);
}
