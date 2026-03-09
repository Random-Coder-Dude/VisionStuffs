/**
 * @file scoreTrajectory.c
 * @brief Scoring function for gradient-descent trajectory optimization.
 */

#include <math.h>
#include "scoreTrajectory.h"
#include "forwardPass.h"
#include "Constants.h"

double scoreTrajectory(SimResult trajectory, Vec3 goalPose, ChassisSpeeds robot) {

    double dx = trajectory.finalPosition.x - goalPose.x;
    double dy = trajectory.finalPosition.y - goalPose.y;

    if (!trajectory.scoreable) {
        double dz = trajectory.maxHeight - goalPose.z - SCORE_UNSCOREABLE_DZ_OFFSET;
        return SCORE_WEIGHT_POSITION * (dx * dx + dy * dy + SCORE_UNSCOREABLE_DZ_WEIGHT * dz * dz);
    }

    double positionCost = dx * dx + dy * dy;

    double sensitivityCost = 0.0;
    SimResult perturbed = calculateTrajectoryCoarse(
        trajectory.RPM + SCORE_RPM_SENSITIVITY_STEP,
        trajectory.hoodAngle,
        trajectory.turretAngle,
        goalPose.z,
        robot
    );

    if (!perturbed.scoreable) {
        sensitivityCost = 1.0;
    } else {
        double ddx     = perturbed.finalPosition.x - trajectory.finalPosition.x;
        double ddy     = perturbed.finalPosition.y - trajectory.finalPosition.y;
        double rawSens = sqrt(ddx * ddx + ddy * ddy) / SCORE_RPM_SENSITIVITY_STEP;
        double normSens = rawSens / (1.0 + rawSens);
        sensitivityCost = normSens * normSens;
    }

    double apexExcess = trajectory.maxHeight - goalPose.z;
    double apexCost   = (apexExcess > 0.0) ? apexExcess * apexExcess : 0.0;

    return SCORE_WEIGHT_POSITION    * positionCost
         + SCORE_WEIGHT_SENSITIVITY * sensitivityCost
         + SCORE_WEIGHT_APEX        * apexCost;
}
