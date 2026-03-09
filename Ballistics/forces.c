/**
 * @file forces.c
 * @brief Implementations of gravitational, drag, and Magnus force models.
 *
 * See forces.h for full mathematical descriptions of each force.
 */

#include <math.h>
#include "forces.h"
#include "Constants.h"

/** Threshold below which speed is treated as zero to avoid divide-by-zero. */
#define SPEED_EPSILON 1e-12

/**
 * @brief Compute the gravitational force on the ball.
 *
 * F_gravity = (0, 0, BALL_MASS * GRAVITY)
 * GRAVITY is negative, so this vector points downward.
 */
Vec3 calculateGravity(void) {
    return createVec3(0.0, 0.0, BALL_MASS * GRAVITY);
}

/**
 * @brief Compute aerodynamic drag.
 *
 * Drag magnitude: |F_drag| = 0.5 * rho * v^2 * Cd * (pi * r^2)
 * Direction: opposite to velocity (hence the negative sign).
 */
Vec3 calculateDrag(Vec3 velocity) {
    double speed = magnitudeVec3(velocity);
    if (speed < SPEED_EPSILON) return createVec3(0.0, 0.0, 0.0);

    double radius        = PROJECTILE_DIAMETER / 2.0;
    double crossSection  = M_PI * radius * radius;            /* A = π r²        */
    double dragMagnitude = -0.5 * AIR_DENSITY * speed * speed /* ½ρv²            */
                           * CD * crossSection;               /* × Cd × A        */

    /* Multiply the (negative) scalar by the unit velocity to oppose motion. */
    return scalarMultVec3(dragMagnitude, normalizeVec3(velocity));
}

/**
 * @brief Compute the Magnus (gyroscopic lift) force.
 *
 * F_magnus = 0.5 * rho * Cl * A * r * (omega × v)
 *
 * The cross product (omega × v) gives the direction of the Magnus force.
 * For backspin (omega = -Y), this produces upward (+Z) lift when moving +X,
 * which extends range and helps the ball drop into the goal from above.
 */
Vec3 calculateMagnus(Vec3 velocity, Vec3 spin) {
    double radius = PROJECTILE_DIAMETER / 2.0;
    double area   = M_PI * radius * radius;

    /* Scalar coefficient for the cross product. */
    double coeff = 0.5 * AIR_DENSITY * CL * area * radius;

    /* Direction of Magnus force: ω × v */
    Vec3 cross = crossVec3(spin, velocity);

    return scalarMultVec3(coeff, cross);
}

/**
 * @brief Sum gravity + drag + Magnus into a net force vector.
 */
Vec3 returnForceVector(Vec3 currentVelocity, Vec3 spinVector) {
    Vec3 gravity = calculateGravity();
    Vec3 drag    = calculateDrag(currentVelocity);
    Vec3 magnus  = calculateMagnus(currentVelocity, spinVector);
    return addVec3(addVec3(gravity, drag), magnus);
}
