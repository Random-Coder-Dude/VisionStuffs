#ifndef FORWARD_PASS
#define FORWARD_PASS
#include "structs.h"
#include "stdbool.h"

typedef struct {
    Vec3 finalPosition;
    double shotTime;
    double maxHeight;
    bool comingFromTop;
} SimResult;

SimResult calculateTrajectory(double rpm, double hoodAngle, double turretAngle, double goalZ, ChassisSpeeds robotVelocity);

#endif
