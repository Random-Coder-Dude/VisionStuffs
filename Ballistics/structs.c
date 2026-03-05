#include <stdio.h>

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

void printVec3(Vec3 a) {
    printf("Vec3: x=%.2f, y=%.2f, z=%.2f\n", a.x, a.y, a.z);
}