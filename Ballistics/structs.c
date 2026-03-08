#include <math.h>
#include "structs.h"

ChassisSpeeds createChassisSpeeds(double vx, double vy, double omega) {
    ChassisSpeeds result;
    result.vx    = vx;
    result.vy    = vy;
    result.omega = omega;
    return result;
}

Vec3 createVec3(double x, double y, double z) {
    Vec3 result;
    result.x = x;
    result.y = y;
    result.z = z;
    return result;
}

Vec3 scalarMultVec3(double scalar, Vec3 a) {
    return createVec3(scalar * a.x, scalar * a.y, scalar * a.z);
}

Vec3 addVec3(Vec3 a, Vec3 b) {
    return createVec3(a.x + b.x, a.y + b.y, a.z + b.z);
}

double magnitudeVec3(Vec3 a) {
    return sqrt((a.x * a.x) + (a.y * a.y) + (a.z * a.z));
}

Vec3 normalizeVec3(Vec3 a) {
    double magnitude = magnitudeVec3(a);
    if (magnitude == 0.0) return createVec3(0.0, 0.0, 0.0);
    return scalarMultVec3(1.0 / magnitude, a);
}

Vec3 crossVec3(Vec3 a, Vec3 b) {
    return createVec3(
        a.y * b.z - a.z * b.y,
        a.z * b.x - a.x * b.z,
        a.x * b.y - a.y * b.x
    );
}

Vec3 lerpVec3(Vec3 a, Vec3 b, double t) {
    return addVec3(a, scalarMultVec3(t, addVec3(b, scalarMultVec3(-1.0, a))));
}

void printVec3(Vec3 a) {
    printf("Vec3: x=%.4f, y=%.4f, z=%.4f\n", a.x, a.y, a.z);
}