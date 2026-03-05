#include <stdio.h>
#include <math.h>

typedef struct {
    double x;
    double y;
    double z;
} Vec3;

Vec3 createVec3(double x, double y, double z) {
    Vec3 result;
    result.x = x;
    result.y = y;
    result.z = z;
    return result; 
}

Vec3 scalarMultVec3(double scalar, Vec3 a) {
    double xTerm = scalar * a.x;
    double yTerm = scalar * a.y;
    double zTerm = scalar * a.z;
    Vec3 result = createVec3(xTerm, yTerm, zTerm);
    return result;
}

Vec3 addVec3(Vec3 a, Vec3 b) {
    double xTerm = a.x + b.x;
    double yTerm = a.y + b.y;
    double zTerm = a.z + b.z;
    Vec3 result = createVec3(xTerm, yTerm, zTerm);
    return result;
}

double magnitudeVec3(Vec3 a) {
    double result = sqrt( (a.x * a.x) + (a.y * a.y) + (a.z * a.z));
    return result;
}

Vec3 normalizeVec3(Vec3 a) {
    double magnitude = magnitudeVec3(a);
    Vec3 result = scalarMultVec3((1/magnitude), a);
    return result;
}

void printVec3(Vec3 a) {
    printf("Vec3: x=%.2f, y=%.2f, z=%.2f\n", a.x, a.y, a.z);
}