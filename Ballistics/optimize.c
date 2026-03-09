/**
 * @file optimize.c
 * @brief Public optimizer API — wraps gradient descent + random restarts.
 *
 * Call optimize() with initial guesses, robot state, and goal position.
 * Returns the best (RPM, hood, turret) found within SCORE_THRESHOLD,
 * or the best found after MAX_RESTARTS attempts.
 */

#include <stdio.h>
#include <math.h>
#include <stdlib.h>
#include <time.h>

#include "optimize.h"
#include "structs.h"
#include "forwardPass.h"
#include "scoreTrajectory.h"

/* =========================================================================
 * Bounds & Hyperparameters
 * ========================================================================= */

#define MIN_RPM          0.0
#define MAX_RPM       6000.0
#define MIN_HOOD         0.0
#define MAX_HOOD        30.0
#define SCORE_THRESHOLD  1.0
#define MAX_RESTARTS      20
#define MAX_ITERATIONS  1000

/* =========================================================================
 * Internal Helpers
 * ========================================================================= */

static double clamp(double value, double min, double max) {
    if (value < min) return min;
    if (value > max) return max;
    return value;
}

static double wrapAngle(double angle) {
    while (angle >  180.0) angle -= 360.0;
    while (angle < -180.0) angle += 360.0;
    return angle;
}

static double evaluate(double rpm, double hood, double turret,
                        Vec3 goalPose, ChassisSpeeds robot)
{
    rpm    = clamp(rpm,  MIN_RPM,  MAX_RPM);
    hood   = clamp(hood, MIN_HOOD, MAX_HOOD);
    turret = wrapAngle(turret);

    SimResult result = calculateTrajectory(rpm, hood, turret, goalPose.z, robot);
    return scoreTrajectory(result, goalPose, robot);
}

static double runOptimizer(double rpm, double hoodAngle, double turretAngle,
                            Vec3 goalPose, ChassisSpeeds robot,
                            double *outRPM, double *outHood, double *outTurret)
{
    double bestRPM    = rpm;
    double bestHood   = hoodAngle;
    double bestTurret = turretAngle;
    double bestScore  = evaluate(rpm, hoodAngle, turretAngle, goalPose, robot);

    const double rpmEps    = 50.0;
    const double hoodEps   =  0.5;
    const double turretEps =  0.5;

    double lr = 5.0;
    const double rpmLrScale    = 100.0;
    const double hoodLrScale   =   1.0;
    const double turretLrScale =   1.0;
    const double lrDecay = 0.5;
    const double lrGrow  = 1.05;
    const double lrMin   = 1e-4;
    const double lrMax   = 200.0;

    for (int i = 0; i < MAX_ITERATIONS; i++) {
        double currentScore = evaluate(rpm, hoodAngle, turretAngle, goalPose, robot);

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

        double mag = sqrt(gradRPM * gradRPM + gradHood * gradHood + gradTurret * gradTurret);
        if (mag < 1e-12) break;

        double uRPM    = gradRPM    / mag;
        double uHood   = gradHood   / mag;
        double uTurret = gradTurret / mag;

        double stepRPM    = lr * rpmLrScale    * uRPM;
        double stepHood   = lr * hoodLrScale   * uHood;
        double stepTurret = lr * turretLrScale * uTurret;

        double newRPM    = clamp(rpm - stepRPM, MIN_RPM, MAX_RPM);
        double newHood   = clamp(hoodAngle - stepHood, MIN_HOOD, MAX_HOOD);
        double newTurret = wrapAngle(turretAngle - stepTurret);
        double newScore  = evaluate(newRPM, newHood, newTurret, goalPose, robot);

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

        rpm         = newRPM;
        hoodAngle   = newHood;
        turretAngle = newTurret;

        if (newScore < bestScore) {
            bestScore  = newScore;
            bestRPM    = rpm;
            bestHood   = hoodAngle;
            bestTurret = turretAngle;
            lr = clamp(lr * lrGrow, lrMin, lrMax);
        }
    }

    *outRPM    = bestRPM;
    *outHood   = bestHood;
    *outTurret = bestTurret;
    return bestScore;
}

/* =========================================================================
 * Public API
 * ========================================================================= */

OptimizeResult optimize(double rpm, double hood,
                        double robotVx, double robotVy, double robotOmega,
                        double goalX, double goalY, double goalZ)
{
    static int seeded = 0;
    if (!seeded) { srand((unsigned int)time(NULL)); seeded = 1; }

    Vec3          goalPose = createVec3(goalX, goalY, goalZ);
    ChassisSpeeds robot    = createChassisSpeeds(robotVx, robotVy, robotOmega);

    /* Turret angle: field-relative bearing to goal, rotated into robot frame. */
    double fieldAngle  = atan2(goalY, goalX);
    double initTurret  = wrapAngle(fieldAngle);

    double bestRPM = 0, bestHood = 0, bestTurret = 0, bestScore = 1e18;

    for (int restart = 0; restart < MAX_RESTARTS; restart++) {
        double seedRPM, seedHood;

        if (restart == 0) {
            seedRPM  = rpm;
            seedHood = hood;
        } else {
            seedRPM  = MIN_RPM  + ((double)rand() / RAND_MAX) * (MAX_RPM  - MIN_RPM);
            seedHood = MIN_HOOD + ((double)rand() / RAND_MAX) * (MAX_HOOD - MIN_HOOD);
            initTurret = wrapAngle(initTurret + ((double)rand() / RAND_MAX) * 30.0 - 15.0);
            printf("Restart %d | rpm=%.0f hood=%.1f turret=%.1f\n",
                   restart, seedRPM, seedHood, initTurret);
        }

        double outRPM, outHood, outTurret;
        double score = runOptimizer(seedRPM, seedHood, initTurret, goalPose, robot,
                                    &outRPM, &outHood, &outTurret);

        printf("Restart %d done | score: %.4f\n", restart, score);

        if (score < bestScore) {
            bestScore  = score;
            bestRPM    = outRPM;
            bestHood   = outHood;
            bestTurret = outTurret;
        }

        if (bestScore <= SCORE_THRESHOLD) break;
    }

    OptimizeResult result;
    result.rpm         = bestRPM;
    result.hoodAngle   = bestHood;
    result.turretAngle = bestTurret;
    result.score       = bestScore;
    return result;
}
