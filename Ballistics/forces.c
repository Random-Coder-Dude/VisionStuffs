#include <math.h>
#include "forces.h"
#include "Constants.h"

Vec3 calculateGravity() {
    return createVec3(0.0, 0.0, gravityForce * ballMass);
}

Vec3 calculateDrag(Vec3 velocity) {
    double velocityMagnitude = magnitudeVec3(velocity);
    if (velocityMagnitude == 0.0) return createVec3(0.0, 0.0, 0.0);

    double radius        = ProjectileDiameter / 2.0;
    double ballArea      = M_PI * radius * radius;
    double dragMagnitude = -0.5 * airDensity * velocityMagnitude * velocityMagnitude * CD * ballArea;

    return scalarMultVec3(dragMagnitude, normalizeVec3(velocity));
}

Vec3 calculateMagnus(Vec3 velocity, Vec3 spin) {
    double radius = ProjectileDiameter / 2.0;
    double area   = M_PI * radius * radius;
    double coeff  = 0.5 * airDensity * CL * area * radius;
    Vec3   cross  = crossVec3(spin, velocity);
    return scalarMultVec3(coeff, cross);
}

Vec3 returnForceVector(Vec3 currentVelocity, Vec3 spinVector) {
    Vec3 gravity = calculateGravity();
    Vec3 drag    = calculateDrag(currentVelocity);
    Vec3 magnus  = calculateMagnus(currentVelocity, spinVector);
    return addVec3(addVec3(gravity, drag), magnus);
}