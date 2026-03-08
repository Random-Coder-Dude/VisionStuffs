#pragma once

#include <stdio.h>

typedef struct {
    double x;
    double y;
    double z;
} Vec3;

typedef struct {
    double vx;
    double vy;
    double omega;
} ChassisSpeeds;

ChassisSpeeds createChassisSpeeds(double vx, double vy, double omega);
Vec3 createVec3(double x, double y, double z);
Vec3 scalarMultVec3(double scalar, Vec3 a);
Vec3 addVec3(Vec3 a, Vec3 b);
double magnitudeVec3(Vec3 a);
Vec3 normalizeVec3(Vec3 a);
Vec3 crossVec3(Vec3 a, Vec3 b);
Vec3 lerpVec3(Vec3 a, Vec3 b, double t);
void printVec3(Vec3 a);