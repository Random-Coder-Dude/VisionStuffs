/**
 * @file main.c
 * @brief Gradient-descent optimizer for FRC ball-shooter parameters.
 *
 * ─────────────────────────────────────────────────────────────────────────
 * OVERVIEW
 * ─────────────────────────────────────────────────────────────────────────
 * Given a fixed robot velocity and a goal position, this program searches for
 * the combination of (RPM, hood angle, turret angle) that minimizes the
 * trajectory score defined in scoreTrajectory.c.
 *
 * The score captures three objectives simultaneously:
 *   1. Accuracy   — land as close to (goalPose.x, goalPose.y) as possible.
 *   2. Arc shape  — prefer lower arcs (less sensitive to wind, easier control).
 *   3. Robustness — prefer shots whose landing point is insensitive to small
 *                   RPM errors (flywheel spin-up uncertainty).
 *
 * ─────────────────────────────────────────────────────────────────────────
 * ALGORITHM: normalized gradient descent with backtracking line search
 * ─────────────────────────────────────────────────────────────────────────
 * At each iteration:
 *   1. Estimate partial derivatives via central finite differences:
 *        ∂score/∂RPM    ≈ [score(RPM+ε) - score(RPM-ε)] / (2ε)
 *        ∂score/∂hood   ≈ [score(hood+ε) - score(hood-ε)] / (2ε)
 *        ∂score/∂turret ≈ [score(turret+ε) - score(turret-ε)] / (2ε)
 *
 *   2. Normalize the gradient vector to unit length so the step direction is
 *      stable regardless of the relative scales of RPM vs. angles.
 *
 *   3. Scale each component by its per-parameter learning-rate multiplier
 *      so that lr=1 corresponds to a physically meaningful step size in each
 *      parameter's natural units.
 *
 *   4. Backtracking line search: repeatedly halve lr until score decreases
 *      (or we hit the minimum lr floor).
 *
 *   5. If the step improved the score, grow lr slightly to accelerate
 *      convergence in smooth regions of the loss surface.
 *
 * ─────────────────────────────────────────────────────────────────────────
 * PARAMETER BOUNDS
 * ─────────────────────────────────────────────────────────────────────────
 *   RPM    : [MIN_RPM,  MAX_RPM]  — clamped hard at every step.
 *   Hood   : [MIN_HOOD, MAX_HOOD] — clamped hard at every step.
 *   Turret : (-180, 180]          — wrapped to keep angle in valid range.
 */

#include <stdio.h>
#include <math.h>

#include "structs.h"
#include "Constants.h"
#include "forwardPass.h"
#include "scoreTrajectory.h"

/* =========================================================================
 * Parameter Bounds
 * ========================================================================= */

/** @brief Minimum flywheel speed to consider (rev/min). */
#define MIN_RPM    0.0

/** @brief Maximum flywheel speed to consider (rev/min). */
#define MAX_RPM 6000.0

/** @brief Minimum hood mechanism angle (degrees). */
#define MIN_HOOD   0.0

/** @brief Maximum hood mechanism angle (degrees). */
#define MAX_HOOD  30.0

/* =========================================================================
 * Utility Functions
 * ========================================================================= */

/**
 * @brief Clamp a value to [min, max].
 * @param value Value to clamp.
 * @param min   Lower bound (inclusive).
 * @param max   Upper bound (inclusive).
 * @return Clamped value.
 */
static double clamp(double value, double min, double max) {
    if (value < min) return min;
    if (value > max) return max;
    return value;
}

/**
 * @brief Wrap an angle to the range (-180, 180].
 *
 * Prevents the turret parameter from drifting to very large values during
 * gradient descent, which could confuse the LUT interpolation.
 *
 * @param angle Angle in degrees (any value).
 * @return Equivalent angle in (-180, 180].
 */
static double wrapAngle(double angle) {
    while (angle >  180.0) angle -= 360.0;
    while (angle < -180.0) angle += 360.0;
    return angle;
}

/**
 * @brief Evaluate the trajectory score for a given set of shooter parameters.
 *
 * Clamps/wraps all inputs to their valid ranges before simulating, so the
 * caller (gradient estimator, line-searcher) can freely extrapolate without
 * producing out-of-range inputs to the physics model.
 *
 * @param rpm          Flywheel speed (rev/min).
 * @param hood         Hood mechanism angle (degrees).
 * @param turret       Turret yaw angle (degrees).
 * @param goalPose     Target position (x, y, z) in meters.
 * @param robot        Robot drivetrain velocity at time of launch.
 * @return Scalar optimization cost (lower = better).
 */
static double evaluate(double rpm, double hood, double turret,
                       Vec3 goalPose, ChassisSpeeds robot)
{
    rpm    = clamp(rpm,  MIN_RPM,  MAX_RPM);
    hood   = clamp(hood, MIN_HOOD, MAX_HOOD);
    turret = wrapAngle(turret);

    SimResult result = calculateTrajectory(rpm, hood, turret, goalPose.z, robot);
    return scoreTrajectory(result, goalPose, robot);
}

/* =========================================================================
 * Main — Gradient Descent Optimizer
 * ========================================================================= */

/**
 * @brief Entry point — runs the gradient-descent optimizer and prints results.
 *
 * Edit the robot and goalPose variables at the top of main() to set up
 * a different shot scenario.
 *
 * @return 0 on success.
 */
int main(void) {

    /* ── Problem setup ──────────────────────────────────────────────────── */

    /** Robot is stationary at the start. */
    ChassisSpeeds robot = createChassisSpeeds(0.0, 0.0, 0.0);

    /**
     * Goal position in field-relative coordinates (meters).
     *   x = 2.0 m forward of the robot
     *   y = 0.0 m (centered)
     *   z = 2.0 m above the floor (top of the goal opening)
     */
    Vec3 goalPose = createVec3(2.0, 0.0, 2.0);

    /* ── Initial parameter guess ─────────────────────────────────────────── */

    double rpm         = 2000.0;  /**< Starting RPM guess          */
    double hoodAngle   =   20.0;  /**< Starting hood angle (deg)   */
    double turretAngle =    0.0;  /**< Starting turret angle (deg) */

    /* Track the best solution seen so far. */
    double bestRPM    = rpm;
    double bestHood   = hoodAngle;
    double bestTurret = turretAngle;
    double bestScore  = evaluate(rpm, hoodAngle, turretAngle, goalPose, robot);

    /* ── Finite-difference step sizes ───────────────────────────────────── */

    /**
     * Central-difference perturbations.
     * Should be large enough to avoid numerical noise but small enough
     * to approximate the local gradient accurately.
     */
    const double rpmEps    = 50.0; /**< ±50 RPM for gradient estimate   */
    const double hoodEps   =  0.5; /**< ±0.5 deg for gradient estimate  */
    const double turretEps =  0.5; /**< ±0.5 deg for gradient estimate  */

    /* ── Learning rate and line-search setup ────────────────────────────── */

    /**
     * Initial learning rate.
     *
     * After gradient normalization, lr directly controls how far we step in
     * "normalized parameter space." Per-parameter scale factors below then
     * convert this into natural units for each parameter.
     */
    double lr = 5.0;

    /**
     * Multipliers that convert normalized gradient components into step sizes
     * in each parameter's natural units.
     *   rpmLrScale = 100   → lr=1 moves RPM by 100 rev/min
     *   hoodLrScale = 1    → lr=1 moves hood by 1 degree
     *   turretLrScale = 1  → lr=1 moves turret by 1 degree
     */
    const double rpmLrScale    = 100.0;
    const double hoodLrScale   =   1.0;
    const double turretLrScale =   1.0;

    /* Backtracking line-search hyperparameters. */
    const double lrDecay = 0.5;   /**< Shrink factor when step is rejected  */
    const double lrGrow  = 1.05;  /**< Growth factor when step is accepted  */
    const double lrMin   = 1e-4;  /**< Floor below which we stop shrinking  */
    const double lrMax   = 200.0; /**< Ceiling on lr after growth           */

    /* ── Main optimization loop ─────────────────────────────────────────── */

    const int MAX_ITERATIONS = 10000;

    for (int i = 0; i < MAX_ITERATIONS; i++) {

        double currentScore = evaluate(rpm, hoodAngle, turretAngle, goalPose, robot);

        /* ── Estimate partial derivatives via central finite differences ──
         *
         * Central differences give O(ε²) accuracy vs. O(ε) for forward
         * differences, which means the gradient estimate is much more
         * accurate for the same step size — important since we're evaluating
         * a noisy simulation function.
         */
        double gradRPM =
            (evaluate(rpm + rpmEps, hoodAngle, turretAngle, goalPose, robot) -
             evaluate(rpm - rpmEps, hoodAngle, turretAngle, goalPose, robot))
            / (2.0 * rpmEps);

        double gradHood =
            (evaluate(rpm, hoodAngle + hoodEps, turretAngle, goalPose, robot) -
             evaluate(rpm, hoodAngle - hoodEps, turretAngle, goalPose, robot))
            / (2.0 * hoodEps);

        double gradTurret =
            (evaluate(rpm, hoodAngle, turretAngle + turretEps, goalPose, robot) -
             evaluate(rpm, hoodAngle, turretAngle - turretEps, goalPose, robot))
            / (2.0 * turretEps);

        /* ── Normalize gradient to unit length ──────────────────────────── */

        double mag = sqrt(gradRPM    * gradRPM    +
                          gradHood   * gradHood   +
                          gradTurret * gradTurret);

        if (mag < 1e-12) {
            /* Gradient has vanished — likely at a flat plateau or true minimum. */
            printf("Iter %4d | score %.6f | gradient vanished, stopping\n",
                   i, currentScore);
            break;
        }

        double uRPM    = gradRPM    / mag;
        double uHood   = gradHood   / mag;
        double uTurret = gradTurret / mag;

        /* ── Compute tentative step ─────────────────────────────────────── */

        double stepRPM    = lr * rpmLrScale    * uRPM;
        double stepHood   = lr * hoodLrScale   * uHood;
        double stepTurret = lr * turretLrScale * uTurret;

        /* Descent: subtract gradient direction. */
        double newRPM    = clamp(rpm - stepRPM, MIN_RPM, MAX_RPM);
        double newHood   = clamp(hoodAngle - stepHood, MIN_HOOD, MAX_HOOD);
        double newTurret = wrapAngle(turretAngle - stepTurret);
        double newScore  = evaluate(newRPM, newHood, newTurret, goalPose, robot);

        /* ── Backtracking line search ────────────────────────────────────── */

        /**
         * Keep halving lr until the step produces a lower score, or until lr
         * hits its minimum (in which case we accept the step anyway and let
         * the next iteration sort it out).
         */
        int lineIter = 0;
        while (newScore >= currentScore && lr > lrMin && lineIter < 30) {
            lr *= lrDecay;

            stepRPM    = lr * rpmLrScale    * uRPM;
            stepHood   = lr * hoodLrScale   * uHood;
            stepTurret = lr * turretLrScale * uTurret;

            newRPM    = clamp(rpm - stepRPM, MIN_RPM, MAX_RPM);
            newHood   = clamp(hoodAngle - stepHood, MIN_HOOD, MAX_HOOD);
            newTurret = wrapAngle(turretAngle - stepTurret);
            newScore  = evaluate(newRPM, newHood, newTurret, goalPose, robot);

            lineIter++;
        }

        /* Accept the step unconditionally after the line search. */
        rpm         = newRPM;
        hoodAngle   = newHood;
        turretAngle = newTurret;

        /* ── Track best and adapt lr ────────────────────────────────────── */

        if (newScore < bestScore) {
            bestScore  = newScore;
            bestRPM    = rpm;
            bestHood   = hoodAngle;
            bestTurret = turretAngle;

            /* Reward: grow lr slightly so we converge faster in smooth regions. */
            lr = clamp(lr * lrGrow, lrMin, lrMax);
        }

        /* ── Periodic diagnostic print ──────────────────────────────────── */

        if (i % 500 == 0 || newScore < bestScore) {
            SimResult dbg = calculateTrajectory(rpm, hoodAngle, turretAngle,
                                                goalPose.z, robot);
            printf("Iter %4d | score %9.4f | rpm %6.1f | hood %5.2f | "
                   "turret %6.2f | land=(%.2f, %.2f) | maxH=%.2f\n",
                   i, newScore, rpm, hoodAngle, turretAngle,
                   dbg.finalPosition.x, dbg.finalPosition.y, dbg.maxHeight);
        }
    }

    /* ── Print final result ──────────────────────────────────────────────── */

    printf("\n=== BEST SOLUTION ===\n");
    printf("Score  : %.6f\n", bestScore);
    printf("RPM    : %.2f\n",  bestRPM);
    printf("Hood   : %.2f deg\n", bestHood);
    printf("Turret : %.2f deg\n", bestTurret);

    /* Show the full trajectory detail for the best solution. */
    SimResult best = calculateTrajectory(bestRPM, bestHood, bestTurret,
                                         goalPose.z, robot);
    printf("\nTrajectory detail:\n");
    printf("  Landing XY : (%.4f, %.4f) m  [goal: (%.2f, %.2f)]\n",
           best.finalPosition.x, best.finalPosition.y,
           goalPose.x, goalPose.y);
    printf("  Max height : %.4f m  [goal z: %.2f]\n", best.maxHeight, goalPose.z);
    printf("  Flight time: %.4f s\n", best.shotTime);
    printf("  Valid      : %s\n", best.valid        ? "yes" : "NO");
    printf("  From top   : %s\n", best.comingFromTop ? "yes" : "NO");

    return 0;
}
