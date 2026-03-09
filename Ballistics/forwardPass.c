/**
 * @file forwardPass.c
 * @brief RK4 numerical trajectory integrator.
 *
 * Uses classical 4th-order Runge-Kutta (RK4) for significantly better accuracy
 * than Euler, especially at longer ranges where Magnus and drag forces compound
 * step errors.
 *
 * RK4 recap — for dy/dt = f(t, y):
 *   k1 = f(t,        y)
 *   k2 = f(t + dt/2, y + dt/2 * k1)
 *   k3 = f(t + dt/2, y + dt/2 * k2)
 *   k4 = f(t + dt,   y + dt   * k3)
 *   y_next = y + (dt/6) * (k1 + 2*k2 + 2*k3 + k4)
 *
 * State vector: [position (Vec3), velocity (Vec3)].
 * Derivative:   [velocity,        acceleration = F_net / m].
 */

#include "forwardPass.h"
#include "initialVelo.h"
#include "forces.h"
#include "Constants.h"

/** Fixed integration timestep (seconds). Smaller = more accurate, slower. */
#define DT 0.001

/* =========================================================================
 * Private RK4 Helper
 * ========================================================================= */

/**
 * @brief Advance [position, velocity] by one RK4 step.
 *
 * Aerodynamic forces depend only on velocity (not position or time), so each
 * k-stage re-evaluates the force model with the velocity estimate from the
 * previous stage.
 *
 * @param[in,out] pos  Position vector, updated in place.
 * @param[in,out] vel  Velocity vector, updated in place.
 * @param spin         Constant spin vector for the step (rad/s).
 * @param dt           Timestep (s).
 */
static void rk4Step(Vec3 *pos, Vec3 *vel, Vec3 spin, double dt) {
    /* k1 — derivative at start of interval */
    Vec3 k1_vel = scalarMultVec3(1.0 / BALL_MASS, returnForceVector(*vel, spin));
    Vec3 k1_pos = *vel;

    /* k2 — derivative at midpoint using k1 estimate */
    Vec3 vel2   = addVec3(*vel, scalarMultVec3(0.5 * dt, k1_vel));
    Vec3 k2_vel = scalarMultVec3(1.0 / BALL_MASS, returnForceVector(vel2, spin));
    Vec3 k2_pos = vel2;

    /* k3 — derivative at midpoint using k2 estimate */
    Vec3 vel3   = addVec3(*vel, scalarMultVec3(0.5 * dt, k2_vel));
    Vec3 k3_vel = scalarMultVec3(1.0 / BALL_MASS, returnForceVector(vel3, spin));
    Vec3 k3_pos = vel3;

    /* k4 — derivative at end of interval using k3 estimate */
    Vec3 vel4   = addVec3(*vel, scalarMultVec3(dt, k3_vel));
    Vec3 k4_vel = scalarMultVec3(1.0 / BALL_MASS, returnForceVector(vel4, spin));
    Vec3 k4_pos = vel4;

    /* Weighted blend: (k1 + 2*k2 + 2*k3 + k4) / 6 */
    *pos = addVec3(*pos, scalarMultVec3(dt / 6.0,
        addVec3(k1_pos, addVec3(scalarMultVec3(2.0, k2_pos),
                        addVec3(scalarMultVec3(2.0, k3_pos), k4_pos)))));

    *vel = addVec3(*vel, scalarMultVec3(dt / 6.0,
        addVec3(k1_vel, addVec3(scalarMultVec3(2.0, k2_vel),
                        addVec3(scalarMultVec3(2.0, k3_vel), k4_vel)))));
}

/* =========================================================================
 * Public API
 * ========================================================================= */

/**
 * @brief Simulate the full ball trajectory using RK4 integration.
 *
 * scoreable is only set true when the ball physically crosses goalZ while
 * descending (vz < 0) before hitting the floor. Both "peaked too low" and
 * "crossed goalZ on the way up" produce scoreable = false.
 *
 * Phase 1 (ascent):
 *   Integrate until vz < 0. Check whether maxHeight >= goalZ.
 *   If not, abort — the ball can never come down through goalZ.
 *
 * Phase 2 (descent):
 *   Continue integrating. The moment position.z crosses goalZ from above,
 *   linearly interpolate to find the exact crossing point and set
 *   scoreable = true. If the ball hits the floor (z < 0) first, scoreable
 *   stays false.
 */
SimResult calculateTrajectory(double rpm, double hoodAngle, double turretAngle,
                              double goalZ, ChassisSpeeds robotVelocity)
{
    Vec3 spin;
    Vec3 position = createVec3(0.0, 0.0, SHOOTER_HEIGHT_OFFSET);
    Vec3 velocity = calculateInitialVelocity(rpm, hoodAngle, turretAngle,
                                             robotVelocity, &spin);

    double time      = 0.0;
    double maxHeight = position.z;
    bool   scoreable = false;

    Vec3 lastPosition = position;
    Vec3 lastVelocity = velocity;

    /* ---- Phase 1: ascent — stop at apex ---- */
    while (position.z >= 0.0) {
        lastPosition = position;
        lastVelocity = velocity;

        rk4Step(&position, &velocity, spin, DT);
        time += DT;

        if (position.z > maxHeight) maxHeight = position.z;

        if (velocity.z < 0.0) {
            /* Apex reached. Only continue to phase 2 if we were high enough. */
            if (maxHeight < goalZ) goto done; /* peaked too low — no chance */
            break;
        }
    }

    /* ---- Phase 2: descent — watch for goalZ crossing ---- */
    while (position.z >= 0.0) {
        lastPosition = position;
        lastVelocity = velocity;

        rk4Step(&position, &velocity, spin, DT);
        time += DT;

        if (position.z > maxHeight) maxHeight = position.z;

        if (position.z <= goalZ) {
            /* Ball crossed goalZ from above — interpolate for sub-step precision. */
            double frac = (goalZ - lastPosition.z) /
                          (position.z - lastPosition.z);
            time     = (time - DT) + frac * DT;
            position = lerpVec3(lastPosition, position, frac);
            velocity = lerpVec3(lastVelocity, velocity, frac);
            scoreable = true;
            break;
        }
    }
    /* If the while loop exits because position.z < 0 (hit the floor before
       crossing goalZ), scoreable remains false. */

done:;
    SimResult result;
    result.scoreable     = scoreable;
    result.finalPosition = position;
    result.shotTime      = time;
    result.maxHeight     = maxHeight;
    result.RPM           = rpm;
    result.hoodAngle     = hoodAngle;
    result.turretAngle   = turretAngle;
    result.spin          = spin;

    return result;
}
