#include <stdio.h>
#include <math.h>

#include "forwardPass.h"
#include "structs.h"
#include "Constants.h"
#include "scoreTrajectory.h"

#define MIN_RPM 0
#define MAX_RPM 6000

#define MIN_HOOD 0
#define MAX_HOOD 90

double clamp(double value, double min, double max) {
    if (value < min) return min;
    if (value > max) return max;
    return value;
}

double wrapAngle(double angle) {
    while (angle > 180) angle -= 360;
    while (angle < -180) angle += 360;
    return angle;
}

double evaluate(double rpm, double hood, double turret,
                Vec3 goalPose, ChassisSpeeds robot)
{
    rpm = clamp(rpm, MIN_RPM, MAX_RPM);
    hood = clamp(hood, MIN_HOOD, MAX_HOOD);
    turret = wrapAngle(turret);

    SimResult result = calculateTrajectory(
        rpm,
        hood,
        turret,
        goalPose.z,
        robot,
        createVec3(0.0, 0.0, 0.0)
    );

    return scoreTrajectory(result, goalPose, robot);
}

int main() {

    ChassisSpeeds robot = createChassisSpeeds(0, 0, 0);
    Vec3 goalPose = createVec3(15, 0, 2.0);

    double rpm = 2000;
    double hoodAngle = 0.0;
    double turretAngle = 0.0;

    double bestRPM = rpm;
    double bestHood = hoodAngle;
    double bestTurret = turretAngle;

    double bestScore = evaluate(rpm, hoodAngle, turretAngle, goalPose, robot);

    /* separate epsilons for better gradients */
    double rpmEps = 200.0;
    double hoodEps = 2.0;
    double turretEps = 2.0;

    double rpmLR = 0.01;
    double hoodLR = 0.2;
    double turretLR = 0.2;

    int iterations = 1000;

    for (int i = 0; i < iterations; i++) {

        double gradRPM =
            (evaluate(rpm + rpmEps, hoodAngle, turretAngle, goalPose, robot) -
             evaluate(rpm - rpmEps, hoodAngle, turretAngle, goalPose, robot))
            / (2 * rpmEps);

        double gradHood =
            (evaluate(rpm, hoodAngle + hoodEps, turretAngle, goalPose, robot) -
             evaluate(rpm, hoodAngle - hoodEps, turretAngle, goalPose, robot))
            / (2 * hoodEps);

        double gradTurret =
            (evaluate(rpm, hoodAngle, turretAngle + turretEps, goalPose, robot) -
             evaluate(rpm, hoodAngle, turretAngle - turretEps, goalPose, robot))
            / (2 * turretEps);

        /* gradient step */
        rpm -= rpmLR * gradRPM;
        hoodAngle -= hoodLR * gradHood;
        turretAngle -= turretLR * gradTurret;

        /* clamp parameters */
        rpm = clamp(rpm, MIN_RPM, MAX_RPM);
        hoodAngle = clamp(hoodAngle, MIN_HOOD, MAX_HOOD);
        turretAngle = wrapAngle(turretAngle);

        double newScore = evaluate(rpm, hoodAngle, turretAngle, goalPose, robot);

        if (newScore < bestScore) {
            bestScore = newScore;
            bestRPM = rpm;
            bestHood = hoodAngle;
            bestTurret = turretAngle;
        }

        printf("Iter %d | score %f\n", i, newScore);
    }

    printf("\nBEST SOLUTION\n");
    printf("Score: %f\n", bestScore);
    printf("RPM: %f\n", bestRPM);
    printf("Hood: %f\n", bestHood);
    printf("Turret: %f\n", bestTurret);

    return 0;
}