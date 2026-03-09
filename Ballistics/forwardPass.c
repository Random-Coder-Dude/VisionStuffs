/**
 * @file forwardPass.c
 * @brief RK4 numerical trajectory integrator.
 *
 * Replaces the original Euler integrator with a classical 4th-order
 * Runge-Kutta (RK4) scheme for significantly improved accuracy, especially
 * at longer ranges where Magnus and drag forces compound step errors.
 *
 * RK4 recap — for dy/dt = f(t, y):
 *   k1 = f(t,        y)
 *   k2 = f(t + dt/2, y + dt/2 * k1)
 *   k3 = f(t + dt/2, y + dt/2 * k2)
 *   k4 = f(t + dt,   y + dt   * k3)
 *   y_next = y + (dt/6) * (k1 + 2*k2 + 2*k3 + k4)
 *
 * Our state vector is [position (Vec3), velocity (Vec3)].
 * The derivative is   [velocity,        acceleration].
 * Acceleration = F_net / m  where F_net = gravity + drag + Magnus.
 *
 * Spin is held constant throughout the flight (rigid-body assumption).
 * In reality spin decays, but the effect on FRC-range shots is small.
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
 * Given current position p and velocity v, computes k1–k4 for both
 * the position derivative (= v) and the velocity derivative (= a = F/m),
 * then blends them in the standard RK4 weighted sum.
 *
 * Because aerodynamic forces depend only on velocity (not position or time),
 * the force evaluation at each k-stage uses the velocity estimate from the
 * previous k-stage.
 *
 * @param[in,out] pos  Position vector, updated in place.
 * @param[in,out] vel  Velocity vector, updated in place.
 * @param spin         Constant spin vector for the step (rad/s).
 * @param dt           Timestep (s).
 */
static void rk4Step(Vec3 *pos, Vec3 *vel, Vec3 spin, double dt) {
    /* --- k1: derivative at the start of the interval --- */
    Vec3 force1 = returnForceVector(*vel, spin);
    Vec3 acc1   = scalarMultVec3(1.0 / BALL_MASS, force1);
    Vec3 k1_pos = *vel;                /* dp/dt = v */
    Vec3 k1_vel = acc1;                /* dv/dt = a */

    /* --- k2: derivative at midpoint, using k1 estimate --- */
    Vec3 vel2   = addVec3(*vel, scalarMultVec3(0.5 * dt, k1_vel));
    Vec3 force2 = returnForceVector(vel2, spin);
    Vec3 acc2   = scalarMultVec3(1.0 / BALL_MASS, force2);
    Vec3 k2_pos = vel2;
    Vec3 k2_vel = acc2;

    /* --- k3: derivative at midpoint, using k2 estimate --- */
    Vec3 vel3   = addVec3(*vel, scalarMultVec3(0.5 * dt, k2_vel));
    Vec3 force3 = returnForceVector(vel3, spin);
    Vec3 acc3   = scalarMultVec3(1.0 / BALL_MASS, force3);
    Vec3 k3_pos = vel3;
    Vec3 k3_vel = acc3;

    /* --- k4: derivative at end of interval, using k3 estimate --- */
    Vec3 vel4   = addVec3(*vel, scalarMultVec3(dt, k3_vel));
    Vec3 force4 = returnForceVector(vel4, spin);
    Vec3 acc4   = scalarMultVec3(1.0 / BALL_MASS, force4);
    Vec3 k4_pos = vel4;
    Vec3 k4_vel = acc4;

    /* --- Weighted blend: (k1 + 2*k2 + 2*k3 + k4) / 6 --- */
    /* Position update */
    Vec3 dPos = scalarMultVec3(dt / 6.0,
        addVec3(k1_pos,
        addVec3(scalarMultVec3(2.0, k2_pos),
        addVec3(scalarMultVec3(2.0, k3_pos),
                k4_pos))));

    /* Velocity update */
    Vec3 dVel = scalarMultVec3(dt / 6.0,
        addVec3(k1_vel,
        addVec3(scalarMultVec3(2.0, k2_vel),
        addVec3(scalarMultVec3(2.0, k3_vel),
                k4_vel))));

    *pos = addVec3(*pos, dPos);
    *vel = addVec3(*vel, dVel);
}

/* =========================================================================
 * Public API
 * ========================================================================= */

/**
 * @brief Simulate the full ball trajectory using RK4 integration.
 *
 * Integration phases:
 *   Phase 1 (ascent):  Integrate until vz < 0 (apex reached).
 *                      Spin is active — Magnus lift matters most here.
 *   Phase 2 (descent): Continue integrating until position.z <= goalZ.
 *                      Spin is kept constant (rigid-body assumption).
 *                      Sub-step linear interpolation pins down the exact
 *                      crossing point for accurate XY landing coordinates.
 *
 * If the ball never reaches goalZ altitude (trajectory too flat), valid=false
 * and comingFromTop=false are set in the returned SimResult.
 */
SimResult calculateTrajectory(double rpm, double hoodAngle, double turretAngle,
                              double goalZ, ChassisSpeeds robotVelocity)
{
    /* Compute the initial velocity and ball spin from shooter parameters.
     * Spin is derived from flywheel surface speed (see initialVelo.c). */
    Vec3 spin;
    Vec3 position = createVec3(0.0, 0.0, SHOOTER_HEIGHT_OFFSET);
    Vec3 velocity = calculateInitialVelocity(rpm, hoodAngle, turretAngle,
                                             robotVelocity, &spin);

    double time      = 0.0;
    double maxHeight = position.z;
    bool   reachedApex = false;

    Vec3 lastPosition = position;
    Vec3 lastVelocity = velocity;

    /* ---- Phase 1: ascent — integrate until the apex ---- */
    while (position.z >= 0.0) {
        lastPosition = position;
        lastVelocity = velocity;

        rk4Step(&position, &velocity, spin, DT);
        time += DT;

        if (position.z > maxHeight) maxHeight = position.z;

        /* The ball has passed the apex when vertical velocity goes negative. */
        if (velocity.z < 0.0) {
            reachedApex = (maxHeight >= goalZ);
            break;
        }
    }

    /* ---- Phase 2: descent — integrate until ball crosses goalZ ---- */
    if (reachedApex) {
        while (position.z >= 0.0) {
            lastPosition = position;
            lastVelocity = velocity;

            /* Spin is maintained constant (rigid-body assumption).
             * Real spin decays slowly, but the effect is negligible
             * for the sub-second flight times seen in FRC. */
            rk4Step(&position, &velocity, spin, DT);
            time += DT;

            if (position.z > maxHeight) maxHeight = position.z;

            /* Ball has crossed the target height — interpolate for precision. */
            if (position.z <= goalZ) {
                /* Fraction of the last step at which z == goalZ. */
                double frac = (goalZ - lastPosition.z) /
                              (position.z - lastPosition.z);
                time     = (time - DT) + frac * DT;
                position = lerpVec3(lastPosition, position, frac);
                velocity = lerpVec3(lastVelocity, velocity, frac);
                break;
            }
        }
    }

    /* ---- Assemble result ---- */
    SimResult result;
    result.valid         = reachedApex;
    result.finalPosition = position;
    result.shotTime      = time;
    result.maxHeight     = maxHeight;
    result.comingFromTop = (maxHeight > goalZ); /* true = descending arc */
    result.RPM           = rpm;
    result.hoodAngle     = hoodAngle;
    result.turretAngle   = turretAngle;
    result.spin          = spin;

    return result;
}
