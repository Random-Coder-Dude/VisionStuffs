/**
 * @file optimize.c
 * @brief Public optimizer API — wraps gradient descent + random restarts.
 */

#include <stdio.h>
#include <math.h>
#include <stdlib.h>
#include <time.h>

#include "optimize.h"
#include "structs.h"
#include "forwardPass.h"
#include "scoreTrajectory.h"
#include "Constants.h"

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
    rpm    = clamp(rpm,  OPT_MIN_RPM,  OPT_MAX_RPM);
    hood   = clamp(hood, OPT_MIN_HOOD, OPT_MAX_HOOD);
    turret = wrapAngle(turret);

    SimResult result = calculateTrajectoryCoarse(rpm, hood, turret, goalPose.z, robot);
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

    double lr = OPT_LR_INIT;

    for (int i = 0; i < OPT_MAX_ITERATIONS; i++) {
        double currentScore = evaluate(rpm, hoodAngle, turretAngle, goalPose, robot);

        double gradRPM =
            (evaluate(rpm + OPT_EPS_RPM, hoodAngle, turretAngle, goalPose, robot) -
             evaluate(rpm - OPT_EPS_RPM, hoodAngle, turretAngle, goalPose, robot))
            / (2.0 * OPT_EPS_RPM);

        double gradHood =
            (evaluate(rpm, hoodAngle + OPT_EPS_HOOD, turretAngle, goalPose, robot) -
             evaluate(rpm, hoodAngle - OPT_EPS_HOOD, turretAngle, goalPose, robot))
            / (2.0 * OPT_EPS_HOOD);

        double gradTurret =
            (evaluate(rpm, hoodAngle, turretAngle + OPT_EPS_TURRET, goalPose, robot) -
             evaluate(rpm, hoodAngle, turretAngle - OPT_EPS_TURRET, goalPose, robot))
            / (2.0 * OPT_EPS_TURRET);

        double mag = sqrt(gradRPM * gradRPM + gradHood * gradHood + gradTurret * gradTurret);
        if (mag < 1e-12) break;

        double uRPM    = gradRPM    / mag;
        double uHood   = gradHood   / mag;
        double uTurret = gradTurret / mag;

        double stepRPM    = lr * OPT_LR_SCALE_RPM    * uRPM;
        double stepHood   = lr * OPT_LR_SCALE_HOOD   * uHood;
        double stepTurret = lr * OPT_LR_SCALE_TURRET * uTurret;

        double newRPM    = clamp(rpm - stepRPM, OPT_MIN_RPM, OPT_MAX_RPM);
        double newHood   = clamp(hoodAngle - stepHood, OPT_MIN_HOOD, OPT_MAX_HOOD);
        double newTurret = wrapAngle(turretAngle - stepTurret);
        double newScore  = evaluate(newRPM, newHood, newTurret, goalPose, robot);

        int lineIter = 0;
        while (newScore >= currentScore && lr > OPT_LR_MIN && lineIter < 30) {
            lr *= OPT_LR_DECAY;
            stepRPM    = lr * OPT_LR_SCALE_RPM    * uRPM;
            stepHood   = lr * OPT_LR_SCALE_HOOD   * uHood;
            stepTurret = lr * OPT_LR_SCALE_TURRET * uTurret;
            newRPM    = clamp(rpm - stepRPM, OPT_MIN_RPM, OPT_MAX_RPM);
            newHood   = clamp(hoodAngle - stepHood, OPT_MIN_HOOD, OPT_MAX_HOOD);
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
            lr = clamp(lr * OPT_LR_GROW, OPT_LR_MIN, OPT_LR_MAX);
        }
    }

    *outRPM    = bestRPM;
    *outHood   = bestHood;
    *outTurret = bestTurret;
    return bestScore;
}

OptimizeResult optimize(double rpm, double hood,
                        double robotVx, double robotVy, double robotOmega,
                        double goalX, double goalY, double goalZ)
{
    static int seeded = 0;
    if (!seeded) { srand((unsigned int)time(NULL)); seeded = 1; }

    Vec3          goalPose = createVec3(goalX, goalY, goalZ);
    ChassisSpeeds robot    = createChassisSpeeds(robotVx, robotVy, robotOmega);

    double initTurret = wrapAngle(atan2(goalY, goalX) * (180.0 / M_PI));

    double bestRPM = 0, bestHood = 0, bestTurret = 0, bestScore = 1e18;

    for (int restart = 0; restart < OPT_MAX_RESTARTS; restart++) {
        double seedRPM, seedHood;

        if (restart == 0) {
            seedRPM  = rpm;
            seedHood = hood;
        } else {
            seedRPM    = OPT_MIN_RPM  + ((double)rand() / RAND_MAX) * (OPT_MAX_RPM  - OPT_MIN_RPM);
            seedHood   = OPT_MIN_HOOD + ((double)rand() / RAND_MAX) * (OPT_MAX_HOOD - OPT_MIN_HOOD);
            initTurret = wrapAngle(initTurret + ((double)rand() / RAND_MAX) * 2.0 * OPT_TURRET_JITTER - OPT_TURRET_JITTER);
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

        if (bestScore <= OPT_SCORE_THRESHOLD) break;
    }

    OptimizeResult result;
    result.rpm         = bestRPM;
    result.hoodAngle   = bestHood;
    result.turretAngle = bestTurret;
    result.score       = bestScore;
    return result;
}
