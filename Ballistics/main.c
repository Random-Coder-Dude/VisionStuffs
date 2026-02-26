#include <stdio.h>
#include <math.h>

struct Vector3d {
    double x;
    double y;
    double z;
};

struct Vector3d gravityForce = {0.0, 0.0, -9.81};
struct Vector3d magnusForce = {0.0, 0.0, 0.0};
struct Vector3d dragForce = {0.0, 0.0, 0.0};
struct Vector3d shotForce = {0.0, 0.0, 0.0};

struct Vector3d position = {0.0, 0.0, 0.0};
struct Vector3d velocity = {0.0, 0.0, 0.0};
struct Vector3d acceleration = {0.0, 0.0, 0.0};

double ballArea;
struct Vector3d spinVector;

double timeStep = 0.01;
double turretAngle = 0.0;
double shooterRPM = 0.0;
double hoodAngle = 0.0;

double ballMass = 0.045;
double ballRadius = 0.02135;
double airDensity = 1.225;
double dragCoefficient = 0.47;
double magnusCoefficient = 0.1;

struct Vector3d createVector(double x, double y, double z) {
    struct Vector3d v;
    v.x = x;
    v.y = y;
    v.z = z;
    return v;
}

struct Vector3d add(struct Vector3d a, struct Vector3d b) {
    struct Vector3d result;
    result.x = a.x + b.x;
    result.y = a.y + b.y;
    result.z = a.z + b.z;
    return result;
}

struct Vector3d scalarMultiply(struct Vector3d v, float scalar) {
    struct Vector3d result;
    result.x = v.x * scalar;
    result.y = v.y * scalar;
    result.z = v.z * scalar;
    return result;
}

struct Vector3d angleToVector2D(double angleDeg) {
    double angleRad = angleDeg * (3.14159 / 180.0);
    return createVector(cos(angleRad), sin(angleRad), 0.0);
}

struct Vector3d vectorCross(struct Vector3d a, struct Vector3d b) {
    struct Vector3d result;
    result.x = a.y * b.z - a.z * b.y;
    result.y = a.z * b.x - a.x * b.z;
    result.z = a.x * b.y - a.y * b.x;
    return result;
}

void printVector(struct Vector3d v) {
    printf("Vector: (%f, %f, %f)\n", v.x, v.y, v.z);
}

double vectorMagnitude(struct Vector3d v) {
    return sqrt(v.x * v.x + v.y * v.y + v.z * v.z);
}

double vectorDot(struct Vector3d a, struct Vector3d b) {
    return a.x * b.x + a.y * b.y + a.z * b.z;
}

struct Vector3d perpendicular2D(struct Vector3d v) {
    return createVector(-v.y, v.x, 0.0);
}

void updateDragForce() {
    double speed = vectorMagnitude(velocity);
    double dragMagnitude = 0.5 * airDensity * speed * speed * dragCoefficient * ballArea;
    struct Vector3d dragDirection = scalarMultiply(velocity, -1.0 / speed);
    dragForce = scalarMultiply(dragDirection, dragMagnitude);
}

void updateMagnusForce() {
    double speed = vectorMagnitude(velocity);
    double magnusMagnitude = 0.5 * airDensity * speed * ballArea * ballRadius * magnusCoefficient;
    struct Vector3d magnusDirection = vectorCross(spinVector, velocity);
    magnusForce = scalarMultiply(magnusDirection, magnusMagnitude);
}

void updateShotForce() {
    shotForce = createVector(0.0, 0.0, 0.0);
    
}

void updateForces() {
    updateDragForce();
    updateMagnusForce();
    updateShotForce();
}

void updateAcceleration() {
    struct Vector3d totalForce = add(add(gravityForce, dragForce), add(magnusForce, shotForce));
    acceleration = scalarMultiply(totalForce, 1.0 / ballMass);
}

void updateVelocity() {
    velocity = add(velocity, scalarMultiply(acceleration, timeStep));
}

void updatePosition() {
    position = add(position, scalarMultiply(velocity, timeStep));
}

int main() {
    double ballArea = 3.14159 * ballRadius * ballRadius;
    struct Vector3d spinVector = perpendicular2D(angleToVector2D(turretAngle));
    struct Vector3d v1 = createVector(1.0, 2.0, 3.0);
    struct Vector3d v2 = createVector(4.0, 5.0, 6.0);
    struct Vector3d v3 = add(v1, v2);
    printVector(v3);
    return 0;
}