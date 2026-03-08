#include "Constants.h"

double LauncherWheelDiameter = 4.0 * 0.0254; // m
double ballMass              = 0.2268;         // kg
double MOI                   = 0.0029264;      // kg*m^2
double ProjectileDiameter    = 0.15;           // m

double gravityForce = -9.81;  // m/s^2
double airDensity   = 1.225;  // kg/m^3
double CD           = 0.47;
double CL           = 0.3;

double shooterOffsetX      = 0.0;  // m
double shooterOffsetY      = 0.0;  // m
double ShooterHeightOffset = 0.5;  // m - height of shooter exit above floor