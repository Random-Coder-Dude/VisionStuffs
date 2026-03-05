#include <stdio.h>
#include <math.h>
#include "structs.h"

int main() {
    double LauncherWheelDiameter = 4.0;
    double ballMass = 0.5;
    double MOI = 10.0;
    double ProjectileDiameter = 5.91;

    int RPM = 1000;
    double Pitch = 0.0;
    double Yaw = 0.0;

    Pitch = Pitch * M_PI / 180.0;
    Yaw = Yaw * M_PI / 180.0;

    double surfaceSpeed = RPM * (LauncherWheelDiameter/2.0) * (0.1047/12.0);
    double speedTransfer = 1.0 / (
        2.0
        + ballMass * ((LauncherWheelDiameter/2.0)*(LauncherWheelDiameter/2.0))/MOI
        + (2.0/5.0) * (ballMass * ((ProjectileDiameter/2.0)*(ProjectileDiameter/2.0))/MOI) * ((LauncherWheelDiameter/ProjectileDiameter)*(LauncherWheelDiameter/ProjectileDiameter))
    );

    double velocityMagnitude = surfaceSpeed * speedTransfer;

    double xTerm = cos(Pitch) * cos(Yaw);
    double yTerm = cos(Pitch) * sin(Yaw);
    double zTerm = sin(Pitch);

    Vec3 ballVelo = createVec3(xTerm, yTerm, zTerm);
    ballVelo = scalarMultVec3(velocityMagnitude, ballVelo);

    printVec3(ballVelo);

    return 0;
}