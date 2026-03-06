#include <stdio.h>
#include <math.h>
#include "structs.h"
#include "Constants.h"

typedef struct {
    int rpm;
    double velocity;
} ShooterEntry;

ShooterEntry shooterLUT[] = {
    {1000, 4.3},
    {2000, 8.5},
    {3000, 12.6},
    {4000, 16.4},
    {5000, 19.9}
};

int LUT_SIZE = 5;

double getVelocityFromRPM(int rpm) {

    if (rpm <= shooterLUT[0].rpm)
        return shooterLUT[0].velocity;

    if (rpm >= shooterLUT[LUT_SIZE-1].rpm)
        return shooterLUT[LUT_SIZE-1].velocity;

    for (int i = 0; i < LUT_SIZE - 1; i++) {

        if (rpm >= shooterLUT[i].rpm && rpm <= shooterLUT[i+1].rpm) {

            double t = (rpm - shooterLUT[i].rpm) /
                       (double)(shooterLUT[i+1].rpm - shooterLUT[i].rpm);

            return shooterLUT[i].velocity +
                   t * (shooterLUT[i+1].velocity - shooterLUT[i].velocity);
        }
    }

    return shooterLUT[LUT_SIZE-1].velocity;
}

Vec3 calculateInitialShotForce(int RPM, double Pitch, double Yaw, ChassisSpeeds robot) {
    Pitch = Pitch * M_PI / 180.0;
    Yaw = Yaw * M_PI / 180.0;

    double velocityMagnitude = getVelocityFromRPM(RPM);
    printf("Inital Velocity Magnitude: %f\n", velocityMagnitude);

    double xTerm = cos(Pitch) * cos(Yaw);
    double yTerm = cos(Pitch) * sin(Yaw);
    double zTerm = sin(Pitch);

    Vec3 ballVelo = createVec3(xTerm, yTerm, zTerm);
    ballVelo = scalarMultVec3(velocityMagnitude, ballVelo);
    Vec3 robotVelocity = createVec3(robot.vx, robot.vy, 0);
    ballVelo = addVec3(robotVelocity, ballVelo);
    Vec3 robotRotationalVelocity = createVec3(-robot.omega * shooterOffsetY, robot.omega * shooterOffsetX , 0.0);
    ballVelo = addVec3(robotRotationalVelocity, ballVelo);

    return ballVelo;
}