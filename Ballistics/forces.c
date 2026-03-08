#include <stdio.h>
#include <math.h>
#include "structs.h"
#include "Constants.h"

Vec3 calculateGravity() {
    Vec3 result = createVec3(0.0, 0.0, gravityForce);
    result = scalarMultVec3(ballMass, result);
    return result;
}

Vec3 calculateDrag(Vec3 velocity) {
    double velocityMagnitude = magnitudeVec3(velocity);
    double radius = ProjectileDiameter / 2.0;
    double ballArea = M_PI * radius * radius;
    double dragMagnitude = -0.5 * airDensity * velocityMagnitude * velocityMagnitude * CD * ballArea;
    Vec3 drag = scalarMultVec3(dragMagnitude, normalizeVec3(velocity));
    return drag;
}

Vec3 calculateMagnus(Vec3 velocity, Vec3 spin) {
    double radius = ProjectileDiameter / 2.0;
    double area = M_PI * radius * radius;

    Vec3 cross = crossVec3(spin, velocity);
    double coeff = 0.5 * airDensity * CL * area * radius;
    Vec3 magnus = scalarMultVec3(coeff, cross);
    return magnus;
}

Vec3 returnForceVector(Vec3 currentVelocity, Vec3 spinVector) {
    Vec3 gravity = calculateGravity();
    Vec3 drag = calculateDrag(currentVelocity);
    Vec3 magnus = calculateMagnus(currentVelocity, spinVector);
    Vec3 result = addVec3(addVec3(gravity, drag), magnus);
    return result;
}