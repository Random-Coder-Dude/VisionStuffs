#include <stdio.h>
#include <math.h>
#include "structs.h"
#include "Constants.h"

Vec3 calculateInitialShotForce(int RPM, double Pitch, double Yaw) {
    Pitch = Pitch * M_PI / 180.0;
    Yaw = Yaw * M_PI / 180.0;

    double omega = 2 * M_PI * RPM / 60.0;
    double surfaceSpeed = omega * (LauncherWheelDiameter/2.0);
    double speedTransfer = 1.0 / (
        2.0
        + ballMass * ((LauncherWheelDiameter/2.0)*(LauncherWheelDiameter/2.0))/MOI
        + (2.0/5.0) * (ballMass * ((ProjectileDiameter/2.0)*(ProjectileDiameter/2.0))/MOI) * ((LauncherWheelDiameter/ProjectileDiameter)*(LauncherWheelDiameter/ProjectileDiameter))
    );

    double velocityMagnitude = surfaceSpeed * speedTransfer;
    printf("Inital Velocity Magnitude: %f\n", velocityMagnitude);

    double xTerm = cos(Pitch) * cos(Yaw);
    double yTerm = cos(Pitch) * sin(Yaw);
    double zTerm = sin(Pitch);

    Vec3 ballVelo = createVec3(xTerm, yTerm, zTerm);
    ballVelo = scalarMultVec3(velocityMagnitude, ballVelo);

    return ballVelo;
}