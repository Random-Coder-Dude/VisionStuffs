/**
 * @file scoreTrajectory.c
 * @brief Scoring function for gradient-descent trajectory optimization.
 */

#include <math.h>
#include "scoreTrajectory.h"
#include "forwardPass.h"
#include "Constants.h"

double scoreTrajectory(SimResult trajectory, Vec3 goalPose, ChassisSpeeds robot) {

    if (!trajectory.scoreable) {
        /* BUG FIX #5: Do NOT use finalPosition when scoreable = false.
         * The forwardPass docs explicitly state finalPosition is meaningless
         * when scoreable = false (ball stopped at floor, apex, etc.).
         * Drive the optimizer toward valid trajectories using only maxHeight,
         * which is always valid, to build a smooth gradient. */
        double dz = trajectory.maxHeight - goalPose.z - SCORE_UNSCOREABLE_DZ_OFFSET;
        return SCORE_WEIGHT_POSITION * SCORE_UNSCOREABLE_DZ_WEIGHT * dz * dz;
    }

    double dx = trajectory.finalPosition.x - goalPose.x;
    double dy = trajectory.finalPosition.y - goalPose.y;
    double positionCost = dx * dx + dy * dy;

    /* BUG FIX #2: SCORE_RPM_SENSITIVITY_STEP was -50.0, making rawSens always
     * negative. The normalization formula rawSens/(1+rawSens) is only well-behaved
     * on [0, ∞) — with a negative input it's non-monotonic and blows up to ±Inf
     * when rawSens == -1. Fix: use fabs() on the delta so the step direction
     * doesn't matter, only the magnitude of landing shift does. */
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
        /* fabs(SCORE_RPM_SENSITIVITY_STEP): normalize by the magnitude of the step
         * regardless of sign, giving m of landing shift per RPM. */
        double rawSens  = sqrt(ddx * ddx + ddy * ddy) / fabs(SCORE_RPM_SENSITIVITY_STEP);
        double normSens = rawSens / (1.0 + rawSens);   /* now always in [0, 1) */
        sensitivityCost = normSens * normSens;
    }

    double apexExcess = trajectory.maxHeight - goalPose.z;
    double apexCost   = (apexExcess > 0.0) ? apexExcess * apexExcess : 0.0;

    return SCORE_WEIGHT_POSITION    * positionCost
         + SCORE_WEIGHT_SENSITIVITY * sensitivityCost
         + SCORE_WEIGHT_APEX        * apexCost;
}
