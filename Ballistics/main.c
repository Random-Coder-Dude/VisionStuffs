#include <stdio.h>
#include <math.h>

#include "structs.h"
#include "Constants.h"
#include "forwardPass.h"
#include "scoreTrajectory.h"

#define MIN_RPM    0.0
#define MAX_RPM 6000.0
#define MIN_HOOD   0.0
#define MAX_HOOD  30.0

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
    rpm    = clamp(rpm, MIN_RPM, MAX_RPM);
    hood   = clamp(hood, MIN_HOOD, MAX_HOOD);
    turret = wrapAngle(turret);

    SimResult result = calculateTrajectory(
        rpm, hood, turret,
        goalPose.z,
        robot,
        createVec3(0.0, 0.0, 0.0)
    );

    return scoreTrajectory(result, goalPose, robot);
}

int main(void) {

    ChassisSpeeds robot    = createChassisSpeeds(0.0, 0.0, 0.0);
    Vec3          goalPose = createVec3(2.0, 0.0, 2.0);

    double rpm         = 2000.0;
    double hoodAngle   = 20.0;
    double turretAngle = 0.0;

    double bestRPM    = rpm;
    double bestHood   = hoodAngle;
    double bestTurret = turretAngle;
    double bestScore  = evaluate(rpm, hoodAngle, turretAngle, goalPose, robot);

    // Finite-difference step sizes — small enough to get accurate local gradients
    const double rpmEps    = 50.0;
    const double hoodEps   =  0.5;
    const double turretEps =  0.5;

    // Single global learning rate applied after gradient normalization.
    // Because we normalize the gradient vector to unit length, this directly
    // controls the step size in "normalized parameter space".
    // RPM is scaled to [0,1] range before normalization so it doesn't dominate.
    double lr = 5.0;

    // Backtracking line-search parameters
    const double lrDecay    = 0.5;
    const double lrGrow     = 1.05;
    const double lrMin      = 1e-4;
    const double lrMax      = 200.0;

    const int iterations = 10000;

    for (int i = 0; i < iterations; i++) {

        double currentScore = evaluate(rpm, hoodAngle, turretAngle, goalPose, robot);

        // Central-difference gradients
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

        // Simple steepest-descent with separate per-parameter learning rates.
        // lr is the base; each param scales it so steps are in natural units.
        // rpmLrScale: lr=1 → 1 RPM step; hoodLrScale: lr=1 → 1 degree step.
        const double rpmLrScale    = 100.0;   // lr=1 moves RPM by 100
        const double hoodLrScale   =   1.0;   // lr=1 moves hood by 1 deg
        const double turretLrScale =   1.0;   // lr=1 moves turret by 1 deg

        // Normalize raw gradient vector to unit length so direction is stable
        double mag = sqrt(gradRPM * gradRPM + gradHood * gradHood + gradTurret * gradTurret);

        if (mag < 1e-12) {
            printf("Iter %4d | score %.6f | gradient vanished, stopping\n", i, currentScore);
            break;
        }

        double uRPM    = gradRPM    / mag;
        double uHood   = gradHood   / mag;
        double uTurret = gradTurret / mag;

        double stepRPM    = lr * rpmLrScale    * uRPM;
        double stepHood   = lr * hoodLrScale   * uHood;
        double stepTurret = lr * turretLrScale * uTurret;

        // Backtracking line search: shrink step until score improves
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
            lr = clamp(lr * lrGrow, lrMin, lrMax); // reward good step
        }

        if (i % 500 == 0 || newScore < bestScore) {
            SimResult dbg = calculateTrajectory(rpm, hoodAngle, turretAngle,
                                                goalPose.z, robot,
                                                createVec3(0,0,0));
            printf("Iter %4d | score %.4f | rpm %.1f | hood %.2f | turret %.2f | land=(%.2f,%.2f) maxH=%.2f\n",
                   i, newScore, rpm, hoodAngle, turretAngle,
                   dbg.finalPosition.x, dbg.finalPosition.y, dbg.maxHeight);
        }
    }

    printf("\n=== BEST SOLUTION ===\n");
    printf("Score  : %.6f\n", bestScore);
    printf("RPM    : %.2f\n", bestRPM);
    printf("Hood   : %.2f deg\n", bestHood);
    printf("Turret : %.2f deg\n", bestTurret);

    return 0;
}