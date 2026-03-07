#include <math.h>
#include "forwardPass.h"
#include "Constants.h"

double scoreTrajectory(SimResult trajectory, Vec3 goalPose, ChassisSpeeds robot) {

    if (!trajectory.comingFromTop) {
        return 1e9;
    }

    // --- Position Error ---
    Vec3 diff = addVec3(trajectory.finalPosition, scalarMultVec3(-1.0, goalPose));
    double posError = magnitudeVec3(diff);

    // --- Height Penalty (prefer lower arcs) ---
    double heightPenalty = trajectory.maxHeight;

    // --- RPM Sensitivity ---
    double rpmStep = 50.0;

    SimResult perturbed = calculateTrajectory(
        trajectory.RPM + rpmStep,
        trajectory.HoodAngle,
        trajectory.TurretAngle,
        goalPose.z,
        robot
    );

    Vec3 deltaPos = addVec3(perturbed.finalPosition, scalarMultVec3(-1.0, trajectory.finalPosition));

    double rpmSensitivity = magnitudeVec3(deltaPos) / rpmStep;

    // --- Weights ---
    double positionWeight = 50.0;
    double heightWeight = 1.0;
    double sensitivityWeight = 100.0;

    return
        positionWeight * posError * posError +
        heightWeight * heightPenalty +
        sensitivityWeight * rpmSensitivity;
}
