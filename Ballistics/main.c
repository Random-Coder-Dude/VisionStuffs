#include <stdio.h>
#include <math.h>

#include "forwardPass.h"
#include "structs.h"
#include "Constants.h"
#include "scoreTrajectory.h"

double evaluate(double rpm, double hood, double turret,
                Vec3 goalPose, ChassisSpeeds robot)
{
    SimResult result = calculateTrajectory(rpm, hood, turret, goalPose.z, robot);
    return scoreTrajectory(result, goalPose, robot);
}

int main() {

    ChassisSpeeds robot = createChassisSpeeds(2, 2, 0);
    Vec3 goalPose = createVec3(2.93, 1.57, 0);

    double rpm = 4500;
    double hoodAngle = 0.0;
    double turretAngle = 0.0;

    double bestRPM = rpm;
    double bestHood = hoodAngle;
    double bestTurret = turretAngle;

    double bestScore = evaluate(rpm, hoodAngle, turretAngle, goalPose, robot);

    double epsilon = 5.0;

    double rpmLR = 0.0005;
    double hoodLR = 0.05;
    double turretLR = 0.05;

    int iterations = 1000;

    for (int i = 0; i < iterations; i++) {

        double base = evaluate(rpm, hoodAngle, turretAngle, goalPose, robot);

        double gradRPM =
            (evaluate(rpm + epsilon, hoodAngle, turretAngle, goalPose, robot) -
             evaluate(rpm - epsilon, hoodAngle, turretAngle, goalPose, robot))
            / (2 * epsilon);

        double gradHood =
            (evaluate(rpm, hoodAngle + epsilon, turretAngle, goalPose, robot) -
             evaluate(rpm, hoodAngle - epsilon, turretAngle, goalPose, robot))
            / (2 * epsilon);

        double gradTurret =
            (evaluate(rpm, hoodAngle, turretAngle + epsilon, goalPose, robot) -
             evaluate(rpm, hoodAngle, turretAngle - epsilon, goalPose, robot))
            / (2 * epsilon);

        rpm -= rpmLR * gradRPM;
        hoodAngle -= hoodLR * gradHood;
        turretAngle -= turretLR * gradTurret;

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
