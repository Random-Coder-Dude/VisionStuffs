#include <stdio.h>
#include <stdbool.h>
#include "initialVelo.h"
#include "forces.h"
#include "Constants.h"

typedef struct {
    Vec3 finalPosition;
    double shotTime;
    double maxHeight;
    bool comingFromTop;
    double RPM;
    double HoodAngle;
    double TurretAngle;
} SimResult;

typedef struct {
    double hoodAngle;
    double pitch;
} HoodAngleEntry;

HoodAngleEntry hoodLUT[] = {
    {0.0, 90.0},
    {5.0, 85.0},
    {10.0, 80.0},
    {15.0, 75.0},
    {20.0, 70.0},
    {25.0, 65.0},
    {30.0, 60.0},
    {35.0, 55.0},
    {40.0, 50.0},
    {45.0, 45.0},
    {50.0, 40.0},
    {55.0, 35.0},
    {60.0, 30.0},
    {65.0, 25.0},
    {70.0, 20.0},
    {75.0, 15.0},
    {80.0, 10.0},
    {85.0, 5.0},
    {90.0, 0.0}
};

const int HOOD_LUT_SIZE = sizeof(hoodLUT)/sizeof(hoodLUT[0]);

double getPitchFromHood(double hoodAngle) {
    if (hoodAngle <= hoodLUT[0].hoodAngle) return hoodLUT[0].pitch;
    if (hoodAngle >= hoodLUT[HOOD_LUT_SIZE-1].hoodAngle) return hoodLUT[HOOD_LUT_SIZE-1].pitch;

    for (int i = 0; i < HOOD_LUT_SIZE - 1; i++) {
        if (hoodAngle >= hoodLUT[i].hoodAngle && hoodAngle <= hoodLUT[i+1].hoodAngle) {
            double t = (hoodAngle - hoodLUT[i].hoodAngle) / (hoodLUT[i+1].hoodAngle - hoodLUT[i].hoodAngle);
            return hoodLUT[i].pitch + t * (hoodLUT[i+1].pitch - hoodLUT[i].pitch);
        }
    }
    return hoodLUT[HOOD_LUT_SIZE-1].pitch;
}

SimResult calculateTrajectory(double rpm, double hoodAngle, double turretAngle, double goalZ, ChassisSpeeds robotVelocity) {
    double pitch = getPitchFromHood(hoodAngle);
    double yaw = turretAngle;

    Vec3 position = createVec3(0.0, 0.0, 0.01); // start slightly above ground
    Vec3 velocity = calculateInitialShotForce(rpm, pitch, yaw, robotVelocity);

    double dt = 0.001;
    double time = 0.0;
    double maxHeight = position.z;
    bool reachedGoalHeight = false;
    Vec3 lastPosition = position;

    // ---- Phase 1: Ascent ----
    while (position.z >= 0.0) {
        Vec3 force = returnForceVector(velocity, createVec3(0.0, 0.0, 0.0));
        Vec3 acceleration = scalarMultVec3(1.0 / ballMass, force);

        velocity = addVec3(velocity, scalarMultVec3(dt, acceleration));
        position = addVec3(position, scalarMultVec3(dt, velocity));

        if (position.z > maxHeight) maxHeight = position.z;

        // Check if we reached goal height
        if (!reachedGoalHeight && position.z >= goalZ) {
            reachedGoalHeight = true;
            // interpolate exact crossing
            double t_corr = (goalZ - lastPosition.z) / (position.z - lastPosition.z) * dt;
            time -= dt;
            time += t_corr;
            position = addVec3(lastPosition, scalarMultVec3(t_corr / dt, addVec3(position, scalarMultVec3(-1.0, lastPosition))));
            break;
        }

        lastPosition = position;
        time += dt;
    }

    // ---- Phase 2: Descent ----
    if (reachedGoalHeight) {
        lastPosition = position;
        while (position.z >= 0.0) {
            Vec3 force = returnForceVector(velocity, createVec3(0.0, 0.0, 0.0));
            Vec3 acceleration = scalarMultVec3(1.0 / ballMass, force);

            velocity = addVec3(velocity, scalarMultVec3(dt, acceleration));
            position = addVec3(position, scalarMultVec3(dt, velocity));

            if (position.z > maxHeight) maxHeight = position.z;

            if (position.z <= goalZ) {
                double t_corr = (goalZ - lastPosition.z) / (position.z - lastPosition.z) * dt;
                time -= dt;
                time += t_corr;
                position = addVec3(lastPosition, scalarMultVec3(t_corr / dt, addVec3(position, scalarMultVec3(-1.0, lastPosition))));
                break;
            }

            lastPosition = position;
            time += dt;
        }
    }

    SimResult result;
    result.finalPosition = position;
    result.shotTime = time;
    result.maxHeight = maxHeight;
    result.comingFromTop = maxHeight > goalZ;
    result.RPM = rpm;
    result.HoodAngle = hoodAngle;
    result.TurretAngle = turretAngle;

    return result;
}