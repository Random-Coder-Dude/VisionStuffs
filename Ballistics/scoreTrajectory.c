/**
 * @file scoreTrajectory.c
 * @brief Continuous, differentiable scoring function for trajectory optimization.
 *
 * The score is structured so that gradient descent in main.c can push the
 * optimizer from any starting point toward a valid, accurate, robust shot.
 *
 * Score formula:
 *   score = positionWeight    * posError²
 *         + heightWeight      * heightExcess²
 *         + sensitivityWeight * rpmSensitivity²
 *         + invalidPenalty
 *
 * All terms are squared so the score surface is smooth (C¹ continuous),
 * which gives finite, useful gradients everywhere — critical for numerical
 * differentiation in the optimizer.
 */

#include <math.h>
#include "scoreTrajectory.h"
#include "Constants.h"

/* =========================================================================
 * Weight Coefficients
 * ========================================================================= */

/** @brief Weight for XY landing position error (dominant term). */
#define WEIGHT_POSITION    100.0

/** @brief Weight for excess arc height above the goal. */
#define WEIGHT_HEIGHT        1.0

/** @brief Weight penalizing shots that are highly sensitive to RPM changes. */
#define WEIGHT_SENSITIVITY   5.0

/* =========================================================================
 * Penalty Constants (large-but-finite for smooth gradient flow)
 * ========================================================================= */

/** @brief Added when the trajectory is physically invalid (never reached goalZ). */
#define PENALTY_INVALID     1e12

/** @brief Added when the ball does not arrive from above (flat / rising shot). */
#define PENALTY_NOT_FROM_TOP 1e9

/* =========================================================================
 * RPM Sensitivity Parameters
 * ========================================================================= */

/** @brief RPM perturbation used to estimate d(landing)/d(RPM) via finite diff. */
#define RPM_SENSITIVITY_STEP 50.0

/**
 * @brief Compute the optimization cost for a simulated trajectory.
 *
 * Implementation notes:
 *
 * 1. **Position error**: sqrt(dx²+dy²) is the XY miss distance at goalZ.
 *    We square it again in the final score formula for a smooth quadratic well.
 *
 * 2. **Height penalty**: Only penalizes arcs *above* the goal height.
 *    Arcs exactly at goalZ height (minimum energy) score zero here.
 *    We square the excess so the gradient points smoothly toward lower arcs.
 *
 * 3. **RPM sensitivity**: A second trajectory is simulated at (RPM + step)
 *    to approximate d(landing)/d(RPM) via forward finite difference.
 *    The raw sensitivity is then passed through the mapping
 *      s' = s / (1 + s)
 *    which squashes [0,∞) → [0,1) — preventing extreme sensitivity values
 *    from dominating the gradient when the optimizer is far from optimum.
 *    We square s' in the final sum for smoothness.
 *
 * 4. **Hard penalties**: Added as plain scalars (not multiplied by position
 *    error) so invalid trajectories still have a consistent score regardless
 *    of where the ball happened to land.
 */
double scoreTrajectory(SimResult trajectory, Vec3 goalPose, ChassisSpeeds robot) {

    /* ---- Validity penalties ---- */
    double invalidPenalty = 0.0;
    if (!trajectory.valid) {
        /* Ball never reached goalZ altitude — completely unusable trajectory. */
        invalidPenalty += PENALTY_INVALID;
    }
    if (!trajectory.comingFromTop) {
        /* Ball arrived from below or level — will hit the rim, not score. */
        invalidPenalty += PENALTY_NOT_FROM_TOP;
    }

    /* ---- XY position error at the goal plane ---- */
    double dx       = trajectory.finalPosition.x - goalPose.x;
    double dy       = trajectory.finalPosition.y - goalPose.y;
    double posError = sqrt(dx * dx + dy * dy); /* Euclidean miss distance (m) */

    /* ---- Excess arc height above the target ---- */
    double heightExcess = trajectory.maxHeight - goalPose.z;
    double heightPenalty = (heightExcess > 0.0) ? heightExcess : 0.0;

    /* ---- RPM sensitivity via forward finite difference ----
     *
     * Simulate a second shot with RPM perturbed by RPM_SENSITIVITY_STEP.
     * The resulting shift in landing XY, divided by the RPM step, gives an
     * approximate d(landing position)/d(RPM) in m/RPM.
     *
     * Note: we re-use the same hoodAngle, turretAngle, robot, and spin
     * direction — only RPM changes.
     */
    double rpmSensitivity;
    SimResult perturbed = calculateTrajectory(
        trajectory.RPM + RPM_SENSITIVITY_STEP,
        trajectory.hoodAngle,
        trajectory.turretAngle,
        goalPose.z,
        robot
    );

    if (!perturbed.valid) {
        /* Perturbation produced an invalid trajectory — assign maximum
         * normalized sensitivity (1.0) to drive the optimizer away. */
        rpmSensitivity = 1.0;
    } else {
        double ddx = perturbed.finalPosition.x - trajectory.finalPosition.x;
        double ddy = perturbed.finalPosition.y - trajectory.finalPosition.y;
        double rawSens = sqrt(ddx * ddx + ddy * ddy) / RPM_SENSITIVITY_STEP;

        /* Sigmoid-like squash to [0, 1): prevents extreme values from
         * overwhelming the position-error term during early optimization. */
        rpmSensitivity = rawSens / (1.0 + rawSens);
    }

    /* ---- Combine into a single scalar cost (all terms quadratic) ---- */
    double score = WEIGHT_POSITION    * posError      * posError
                 + WEIGHT_HEIGHT      * heightPenalty * heightPenalty
                 + WEIGHT_SENSITIVITY * rpmSensitivity * rpmSensitivity
                 + invalidPenalty;

    return score;
}
