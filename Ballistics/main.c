#include <stdio.h>
#include "initialVelo.h"
#include "forces.h"

int main() {
    double rpm = 2000;
    double pitch = 45;
    double yaw = 0;

    Vec3 vector = calculateInitialShotForce(rpm, pitch, yaw);
    Vec3 force = returnForceVector(vector, createVec3(0.0, 0.0, 0.0));
    printVec3(vector);
    printVec3(force);
}