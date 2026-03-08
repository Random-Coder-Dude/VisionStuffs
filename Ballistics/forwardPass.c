#include "forwardPass.h"
#include "initialVelo.h"
#include "forces.h"
#include "Constants.h"

typedef struct {
    double hoodAngle;
    double pitch;
} HoodAngleEntry;

static HoodAngleEntry hoodLUT[] = {
    { 0.0, 90.0}, { 5.0, 85.0}, {10.0, 80.0}, {15.0, 75.0}, {20.0, 70.0},
    {25.0, 65.0}, {30.0, 60.0}, {35.0, 55.0}, {40.0, 50.0}, {45.0, 45.0},
    {50.0, 40.0}, {55.0, 35.0}, {60.0, 30.0}, {65.0, 25.0}, {70.0, 20.0},
    {75.0, 15.0}, {80.0, 10.0}, {85.0,  5.0}, {90.0,  0.0}
};

static const int HOOD_LUT_SIZE = sizeof(hoodLUT) / sizeof(hoodLUT[0]);

static double getPitchFromHood(double hoodAngle) {
    if (hoodAngle <= hoodLUT[0].hoodAngle)               return hoodLUT[0].pitch;
    if (hoodAngle >= hoodLUT[HOOD_LUT_SIZE-1].hoodAngle) return hoodLUT[HOOD_LUT_SIZE-1].pitch;

    for (int i = 0; i < HOOD_LUT_SIZE - 1; i++) {
        if (hoodAngle >= hoodLUT[i].hoodAngle && hoodAngle <= hoodLUT[i+1].hoodAngle) {
            double t = (hoodAngle - hoodLUT[i].hoodAngle) /
                       (hoodLUT[i+1].hoodAngle - hoodLUT[i].hoodAngle);
            return hoodLUT[i].pitch + t * (hoodLUT[i+1].pitch - hoodLUT[i].pitch);
        }
    }
    return hoodLUT[HOOD_LUT_SIZE-1].pitch;
}

SimResult calculateTrajectory(double rpm, double hoodAngle, double turretAngle,
                              double goalZ, ChassisSpeeds robotVelocity, Vec3 spin)
{
    double pitch = getPitchFromHood(hoodAngle);
    double yaw   = turretAngle;

    Vec3 position = createVec3(0.0, 0.0, ShooterHeightOffset);
    Vec3 velocity = calculateInitialShotForce(rpm, pitch, yaw, robotVelocity);

    const double dt  = 0.001;
    double time      = 0.0;
    double maxHeight = position.z;
    bool   reachedGoalHeightAscent = false;

    Vec3 lastPosition = position;
    Vec3 lastVelocity = velocity;

    // ---- Phase 1: fly until apex (velocity.z flips negative) ----
    while (position.z >= 0.0) {
        Vec3 force        = returnForceVector(velocity, spin);
        Vec3 acceleration = scalarMultVec3(1.0 / ballMass, force);

        lastPosition = position;
        lastVelocity = velocity;

        time     += dt;
        velocity  = addVec3(velocity, scalarMultVec3(dt, acceleration));
        position  = addVec3(position, scalarMultVec3(dt, velocity));

        if (position.z > maxHeight) maxHeight = position.z;

        // Reached apex — switch to descent phase
        if (velocity.z < 0.0) {
            reachedGoalHeightAscent = (maxHeight >= goalZ);
            break;
        }
    }

    // ---- Phase 2: descent until ball crosses goalZ downward ----
    if (reachedGoalHeightAscent) {
        Vec3 noSpin = createVec3(0.0, 0.0, 0.0);

        while (position.z >= 0.0) {
            Vec3 force        = returnForceVector(velocity, noSpin);
            Vec3 acceleration = scalarMultVec3(1.0 / ballMass, force);

            lastPosition = position;
            lastVelocity = velocity;

            time     += dt;
            velocity  = addVec3(velocity, scalarMultVec3(dt, acceleration));
            position  = addVec3(position, scalarMultVec3(dt, velocity));

            if (position.z > maxHeight) maxHeight = position.z;

            if (position.z <= goalZ) {
                double frac = (goalZ - lastPosition.z) / (position.z - lastPosition.z);
                time     = (time - dt) + frac * dt;
                position = lerpVec3(lastPosition, position, frac);
                velocity = lerpVec3(lastVelocity, velocity, frac);
                break;
            }
        }
    }

    SimResult result;
    result.valid         = reachedGoalHeightAscent;
    result.finalPosition = position;
    result.shotTime      = time;
    result.maxHeight     = maxHeight;
    result.comingFromTop = maxHeight > goalZ;
    result.RPM           = rpm;
    result.HoodAngle     = hoodAngle;
    result.TurretAngle   = turretAngle;
    result.spin          = spin;

    return result;
}