#include <math.h>
#include "initialVelo.h"
#include "Constants.h"

typedef struct {
    double rpm;
    double velocity;
} ShooterEntry;

static ShooterEntry shooterLUT[] = {
    {1000.0,  4.3},
    {2000.0,  8.5},
    {3000.0, 12.6},
    {4000.0, 16.4},
    {5000.0, 19.9}
};

static int LUT_SIZE = sizeof(shooterLUT) / sizeof(shooterLUT[0]);

static double getVelocityFromRPM(double rpm) {
    if (rpm <= shooterLUT[0].rpm)
        return shooterLUT[0].velocity;

    if (rpm >= shooterLUT[LUT_SIZE - 1].rpm)
        return shooterLUT[LUT_SIZE - 1].velocity;

    for (int i = 0; i < LUT_SIZE - 1; i++) {
        if (rpm >= shooterLUT[i].rpm && rpm <= shooterLUT[i + 1].rpm) {
            double t = (rpm - shooterLUT[i].rpm) /
                       (shooterLUT[i + 1].rpm - shooterLUT[i].rpm);
            return shooterLUT[i].velocity +
                   t * (shooterLUT[i + 1].velocity - shooterLUT[i].velocity);
        }
    }

    return shooterLUT[LUT_SIZE - 1].velocity;
}

Vec3 calculateInitialShotForce(double RPM, double pitch, double yaw, ChassisSpeeds robot) {
    pitch = pitch * M_PI / 180.0;
    yaw   = yaw   * M_PI / 180.0;

    double velocityMagnitude = getVelocityFromRPM(RPM);

    Vec3 ballVelo = createVec3(
        cos(pitch) * cos(yaw),
        cos(pitch) * sin(yaw),
        sin(pitch)
    );
    ballVelo = scalarMultVec3(velocityMagnitude, ballVelo);

    Vec3 robotVelocity = createVec3(robot.vx, robot.vy, 0.0);
    ballVelo = addVec3(ballVelo, robotVelocity);

    Vec3 robotRotationalVelocity = createVec3(
        -robot.omega * shooterOffsetY,
         robot.omega * shooterOffsetX,
         0.0
    );
    ballVelo = addVec3(ballVelo, robotRotationalVelocity);

    return ballVelo;
}