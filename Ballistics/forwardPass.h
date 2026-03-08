#pragma once

#include <stdbool.h>
#include "structs.h"

typedef struct {
    Vec3   finalPosition;
    double shotTime;
    double maxHeight;
    bool   comingFromTop;
    bool   valid;
    double RPM;
    double HoodAngle;
    double TurretAngle;
    Vec3   spin;
} SimResult;

SimResult calculateTrajectory(double rpm, double hoodAngle, double turretAngle,
                              double goalZ, ChassisSpeeds robotVelocity, Vec3 spin);