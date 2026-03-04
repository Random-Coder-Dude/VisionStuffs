#include <stdio.h>
#include <math.h>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

// Conventions:
// +x = forewards
// +y = left
// +z = up
// CCw is +ve
// CW is -ve
// Right hand rule

double wheelMass = 2.26796; // Kg
double ballMass = 0.226796; // Kg
double wheelRadius = 0.1016; // Meters
double turretOffsetX = 0; // Meters
double turretOffsetY = 0; // Meters
double shooterOffsetX = 0; // Meters
double shooterOffsetY = 0; // Meters
double shooterOffsetZ = 0; // Meters
double robotvX = 0.0; // Meters
double robotvY = 0.0; // Meters
double robotRotation = 0.0; // Rad/sec
double turretRotation = 0.0; // Rad/sec
double gravityForce = -9.81; // Meteres/sec
double airDensity = 1.225; // Kg/meters^3
double dragCoefficent = 0.6; // Dimensionless
double ballRadius = 0.075; // Meters
double simTimeStep = 0.001; // seconds
double shooterEffeciency = 1; // Dimensionless

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

Vec3 create(double x, double y, double z) {
    Vec3 result = {x, y, z};
    return result;
}

ChassisSpeeds createChassisSpeeds(double x, double y, double omega) {
    ChassisSpeeds result = {x, y, omega};
    return result;
}

Vec3 add(Vec3 a, Vec3 b) {
    double xTerm = a.x + b.x;
    double yTerm = a.y + b.y;
    double zTerm = a.z + b.z;
    Vec3 result = create(xTerm, yTerm, zTerm);
    return result;
}

Vec3 negate(Vec3 a) {
    double xTerm = -a.x;
    double yTerm = -a.y;
    double zTerm = -a.z;
    Vec3 result = create(xTerm, yTerm, zTerm);
    return result;
}

Vec3 scalarMult(double scalar, Vec3 b) {
    double xTerm = scalar * b.x;
    double yTerm = scalar * b.y;
    double zTerm = scalar * b.z;
    Vec3 result = create(xTerm, yTerm, zTerm);
    return result;
}

double dotProduct(Vec3 a, Vec3 b) {
    double xTerm = a.x * b.x;
    double yTerm = a.y * b.y;
    double zTerm = a.z * b.z;
    double result = xTerm + yTerm + zTerm;
    return result;
}

double magnitude(Vec3 a) {
    double xTerm = a.x * a.x;
    double yTerm = a.y * a.y;
    double zTerm = a.z * a.z;
    double result = sqrt(xTerm + yTerm + zTerm);
    return result;
}

Vec3 normalize(Vec3 a) {
    double mag = magnitude(a);
    if (mag < 1e-12) {
        Vec3 result = create(0.0, 0.0, 0.0);
        return result;
    }
    Vec3 result = scalarMult((1.0/mag), a);
    return result;
}

void printVec3(Vec3 a) {
    printf("Vec3: [%.3f, %.3f, %.3f]\n", a.x, a.y, a.z);
}

Vec3 calculateInitialVelocity(double RPM, double pitchRadians, double yawRadians, ChassisSpeeds robot, ChassisSpeeds turret) {
    //Calculated from Julia's Calc
    double rotation = (2*M_PI*RPM)/60;
    double effeciency = (wheelMass)/(wheelMass+((7.0/5.0) * ballMass));
    double velocityMagnitude = rotation * wheelRadius * 0.5 * effeciency;

    //Our homemade direction vector
    double termX = cos(pitchRadians) * cos(yawRadians);
    double termY = cos(pitchRadians) * sin(yawRadians);
    double termZ = sin(pitchRadians);
    Vec3 direction = create(termX, termY, termZ);
    //Just in case (floating point error)
    direction = normalize(direction);
    Vec3 initialVelocity = scalarMult(velocityMagnitude, direction);
    initialVelocity = scalarMult(shooterEffeciency, initialVelocity);

    //Shooting on move part
    //Translation
    double Vx = robot.x;
    double Vy = robot.y;
    Vec3 robotTranslation = create(Vx, Vy, 0);
    initialVelocity = add(initialVelocity, robotTranslation);

    //Robot Rotation
    Vx = -1 * robot.omega * turretOffsetY;
    Vy = 1 * robot.omega * turretOffsetX;
    Vec3 robotRotationalTranslation = create (Vx, Vy, 0);
    initialVelocity = add(initialVelocity, robotRotationalTranslation);

    //Turret Rotation
    Vx = -1 * turret.omega * shooterOffsetY;
    Vy =  1 * turret.omega * shooterOffsetX;
    Vec3 turretRotationalTranslation = create(Vx, Vy, 0);
    initialVelocity = add(initialVelocity, turretRotationalTranslation);

    return initialVelocity;
}

Vec3 calculateDrag(Vec3 velocity, double ballArea) {
    double velocityMagnitude = magnitude(velocity);
    double dragMagnitude = -0.5 * airDensity * pow(velocityMagnitude, 2) * dragCoefficent * ballArea;
    Vec3 direction = normalize(velocity);
    Vec3 result = scalarMult(dragMagnitude, direction);
    return result;
}

Vec3 calculateAcceleration(Vec3 summedForces) {
    Vec3 result = scalarMult((1.0/ballMass), summedForces);
    return result;
}

Vec3 calculateVelocity(Vec3 currentAccel, Vec3 currentVelo) {
    Vec3 deltaV = scalarMult(simTimeStep, currentAccel);
    return add(currentVelo, deltaV);
}

Vec3 calculatePosistion(Vec3 currentVelo, Vec3 currentPosistion) {
    Vec3 deltaP = scalarMult(simTimeStep, currentVelo);
    return add(currentPosistion, deltaP);
}

simulationResult simulateFinalPosistion(int RPM, double m_pitch, double m_yaw, double goalZ) {
    double pitch = m_pitch * (M_PI/180);
    double yaw = m_yaw * (M_PI/180);

    Vec3 gravity = create(0, 0, ballMass * gravityForce);
    double ballArea = M_PI * ballRadius * ballRadius;

    ChassisSpeeds robot = createChassisSpeeds(robotvX, robotvY, robotRotation);
    ChassisSpeeds turret = createChassisSpeeds(0, 0, turretRotation);

    Vec3 ballVelo = calculateInitialVelocity(RPM, pitch, yaw, robot, turret);
    Vec3 ballPos  = create(shooterOffsetX, shooterOffsetY, shooterOffsetZ + 0.001);

    simulationResult result;
    result.maxHeight  = ballPos.z;
    result.flightTime = 0.0;

    // Phase 1: wait until ball rises above goalZ
    while (ballPos.z < goalZ) {
        Vec3 dragForce  = calculateDrag(ballVelo, ballArea);
        Vec3 totalForce = add(dragForce, gravity);
        Vec3 accel      = calculateAcceleration(totalForce);
        ballVelo        = calculateVelocity(accel, ballVelo);
        ballPos         = calculatePosistion(ballVelo, ballPos);
        result.flightTime += simTimeStep;

        if (ballPos.z > result.maxHeight) result.maxHeight = ballPos.z;

        if (ballPos.z <= 0) {
            result.posistion    = ballPos;
            result.velocity     = ballVelo;
            result.acceleration = accel;
            result.maxHeight    = 0;
            result.flightTime   = 0;
            return result;
        }
    }

    // Phase 2: simulate until ball comes back down through goalZ
    while (ballPos.z > goalZ) {
        Vec3 dragForce  = calculateDrag(ballVelo, ballArea);
        Vec3 totalForce = add(dragForce, gravity);
        Vec3 accel      = calculateAcceleration(totalForce);
        ballVelo        = calculateVelocity(accel, ballVelo);
        ballPos         = calculatePosistion(ballVelo, ballPos);
        result.flightTime += simTimeStep;

        if (ballPos.z > result.maxHeight) result.maxHeight = ballPos.z;
    }

    result.posistion    = ballPos;
    result.velocity     = ballVelo;
    result.acceleration = create(0, 0, 0);
    return result;
}