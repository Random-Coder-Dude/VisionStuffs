#ifndef FORWARD_PASS
#define FORWARD_PASS
#include "structs.h"
#include "stdbool.h"

typedef struct {
    Vec3 finalPosition;
    double shotTime;
    double maxHeight;
    bool comingFromTop;
    double RPM;
    double HoodAngle;
    double TurretAngle;
    Vec3 spin;
} SimResult;

SimResult calculateTrajectory(double rpm, double hoodAngle, double turretAngle, double goalZ, ChassisSpeeds robotVelocity, Vec3 spin);

#endif
