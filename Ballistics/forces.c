#include <stdio.h>
#include <math.h>
#include "structs.h"
#include "Constants.h"

Vec3 calculateGravity() {
    Vec3 result = createVec3(0.0, 0.0, 1);
    result = scalarMultVec3(gravityForce, result);
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

Vec3 returnForceVector(Vec3 currentVelocity, Vec3 spinVector) {
    Vec3 gravity = calculateGravity();
    Vec3 drag = calculateDrag(currentVelocity);
    Vec3 result = addVec3(gravity, drag);
    return result;
}