#ifndef FORWARD_PASS_H
#define FORWARD_PASS_H

typedef struct {
    double x;
    double y;
    double z;
} Vec3;

typedef struct {
    double x;
    double y;
    double omega;
} ChassisSpeeds;

typedef struct {
    Vec3 posistion;
    Vec3 velocity;
    Vec3 acceleration;
    double maxHeight;
    double flightTime;
} simulationResult;

extern double wheelMass;
extern double ballMass;
extern double wheelRadius;
extern double turretOffsetX;
extern double turretOffsetY;
extern double shooterOffsetX;
extern double shooterOffsetY;
extern double shooterOffsetZ;
extern double robotvX;
extern double robotvY;
extern double robotRotation;
extern double turretRotation;
extern double gravityForce;
extern double airDensity;
extern double dragCoefficent;
extern double ballRadius;
extern double simTimeStep;
extern double shooterEffeciency;

Vec3 create(double x, double y, double z);
ChassisSpeeds createChassisSpeeds(double x, double y, double omega);
Vec3 add(Vec3 a, Vec3 b);
Vec3 negate(Vec3 a);
Vec3 scalarMult(double scalar, Vec3 b);
double dotProduct(Vec3 a, Vec3 b);
double magnitude(Vec3 a);
Vec3 normalize(Vec3 a);
void printVec3(Vec3 a);
Vec3 calculateInitialVelocity(double RPM, double pitchRadians, double yawRadians, ChassisSpeeds robot, ChassisSpeeds turret);
Vec3 calculateDrag(Vec3 velocity, double ballArea);
Vec3 calculateAcceleration(Vec3 summedForces);
Vec3 calculateVelocity(Vec3 currentAccel, Vec3 currentVelo);
Vec3 calculatePosistion(Vec3 currentVelo, Vec3 currentPosistion);
simulationResult simulateFinalPosistion(int RPM, double m_pitch, double m_yaw, double goalZ);

#endif