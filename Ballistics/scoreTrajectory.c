/**
 * @file scoreTrajectory.c
 * @brief Scoring function for gradient-descent trajectory optimization.
 *
 * Two modes depending on trajectory.scoreable:
 *
 * ── scoreable = true (ball descended through goalZ) ──────────────────────
 *
 *   score = WEIGHT_POSITION * (dx² + dy²)
 *         + WEIGHT_SENSITIVITY * rpmSensitivity²
 *
 * ── scoreable = false (peaked too low, hit floor short, etc.) ────────────
 *
 *   score = WEIGHT_POSITION * (dx² + dy² + dz²)
 *
 *   where dz = maxHeight - goalZ  (negative when ball peaked below goal).
 *   Full 3-D distance gives a smooth gradient toward valid trajectories
 *   from any starting point — no hard penalty cliffs.
 */

#include <math.h>
#include "scoreTrajectory.h"
#include "Constants.h"

#define WEIGHT_POSITION      100.0
#define WEIGHT_SENSITIVITY     5.0
#define RPM_SENSITIVITY_STEP  50.0

/**
 * @brief Compute the optimization cost for a simulated trajectory.
 *
 * @param trajectory  Result from calculateTrajectory().
 * @param goalPose    (x, y) = desired landing point (m), z = target height (m).
 * @param robot       Robot velocity forwarded to the sensitivity simulation.
 * @return Non-negative scalar cost. Lower = better.
 */
double scoreTrajectory(SimResult trajectory, Vec3 goalPose, ChassisSpeeds robot) {

    double dx = trajectory.finalPosition.x - goalPose.x;
    double dy = trajectory.finalPosition.y - goalPose.y;

    if (!trajectory.scoreable) {
        /* Ball never made a valid descending crossing of goalZ.
         * Use 3-D distance so the gradient still points toward higher,
         * longer arcs rather than a flat featureless penalty. */
        double dz = trajectory.maxHeight - goalPose.z;
        return WEIGHT_POSITION * (dx * dx + dy * dy + 10.0 * dz * dz);
    }

    /* ── Valid shot: 2-D XY accuracy + RPM robustness ── */

    double positionCost = dx * dx + dy * dy;

    /* Forward finite difference: how much does landing shift per RPM unit? */
    double sensitivityCost = 0.0;
    SimResult perturbed = calculateTrajectory(
        trajectory.RPM + RPM_SENSITIVITY_STEP,
        trajectory.hoodAngle,
        trajectory.turretAngle,
        goalPose.z,
        robot
    );

    if (!perturbed.scoreable) {
        /* Perturbation fell off the edge of the valid region — maximally sensitive. */
        sensitivityCost = 1.0;
    } else {
        double ddx     = perturbed.finalPosition.x - trajectory.finalPosition.x;
        double ddy     = perturbed.finalPosition.y - trajectory.finalPosition.y;
        double rawSens = sqrt(ddx * ddx + ddy * ddy) / RPM_SENSITIVITY_STEP;

        /* Squash [0,∞) → [0,1) so outliers don't swamp the position term. */
        double normSens = rawSens / (1.0 + rawSens);
        sensitivityCost = normSens * normSens;
    }

    return WEIGHT_POSITION    * positionCost
         + WEIGHT_SENSITIVITY * sensitivityCost;
}
