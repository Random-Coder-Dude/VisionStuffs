#include <math.h>
#include "scoreTrajectory.h"
#include "Constants.h"

// Continuous scoring function for shooter trajectory optimization
double scoreTrajectory(SimResult trajectory, Vec3 goalPose, ChassisSpeeds robot) {

    // --- Soft penalty for invalid trajectories ---
    // Use smooth, additive penalties instead of hard stops
    double invalidPenalty = 0.0;
    if (!trajectory.valid) {
        invalidPenalty += 1e12;           // trajectory is invalid
    }
    if (!trajectory.comingFromTop) {
        invalidPenalty += 1e9;         // trajectory doesn't come from top
    }

    // --- Position error in XY plane at z = goalZ ---
    double dx = trajectory.finalPosition.x - goalPose.x;
    double dy = trajectory.finalPosition.y - goalPose.y;
    double posError = sqrt(dx*dx + dy*dy);

    // --- Height penalty: prefer lower arcs ---
    // Smooth penalty for excess height above the target plane
    double heightExcess = trajectory.maxHeight - goalPose.z;
    double heightPenalty = (heightExcess > 0.0) ? sqrt(heightExcess) : 0.0;

    // --- RPM sensitivity (m / RPM) ---
    // Penalize shots that shift too much with small RPM changes
    const double rpmStep = 50.0;
    SimResult perturbed = calculateTrajectory(
        trajectory.RPM + rpmStep,
        trajectory.HoodAngle,
        trajectory.TurretAngle,
        goalPose.z,
        robot,
        trajectory.spin
    );

    double rpmSensitivity;
    if (!perturbed.valid) {
        rpmSensitivity = 1.0;  // highly sensitive if perturbation fails
    } else {
        double ddx = perturbed.finalPosition.x - trajectory.finalPosition.x;
        double ddy = perturbed.finalPosition.y - trajectory.finalPosition.y;
        rpmSensitivity = sqrt(ddx*ddx + ddy*ddy) / rpmStep;

        // Smoothly cap extreme sensitivity using a sigmoid-like mapping
        rpmSensitivity = rpmSensitivity / (1.0 + rpmSensitivity);
    }

    // --- Weight coefficients ---
    const double positionWeight    = 100.0;  // prioritize hitting XY plane accurately
    const double heightWeight      = 1.0;    // moderate penalty for high arcs
    const double sensitivityWeight = 5.0;   // discourage unstable shots

    // --- Combined score (always continuous) ---
    double score = positionWeight    * posError * posError +
                   heightWeight      * heightPenalty * heightPenalty +
                   sensitivityWeight * rpmSensitivity * rpmSensitivity +
                   invalidPenalty;

    return score;
}