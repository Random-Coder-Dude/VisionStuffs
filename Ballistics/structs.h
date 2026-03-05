#ifndef STRUCT_H
#define STRUCT_H

typedef struct {
    double x;
    double y;
    double z;
} Vec3;

Vec3 createVec3(double x, double y, double z);
Vec3 scalarMultVec3(double scalar, Vec3 a);
void printVec3(Vec3 a);

#endif
