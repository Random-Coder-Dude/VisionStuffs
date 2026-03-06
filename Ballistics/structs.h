#ifndef STRUCT_H
#define STRUCT_H

typedef struct {
    double x;
    double y;
    double z;
} Vec3;

Vec3 createVec3(double x, double y, double z);
Vec3 scalarMultVec3(double scalar, Vec3 a);
Vec3 addVec3(Vec3 a, Vec3 b);
double magnitudeVec3(Vec3 a);
Vec3 normalizeVec3(Vec3 a);
Vec3 crossVec3(Vec3 a, Vec3 b);
void printVec3(Vec3 a);

#endif
