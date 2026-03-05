#include <stdio.h>
#include "initialVelo.h"

int main() {
    double rpm = 2000;
    double pitch = 45;
    double yaw = 0;

    Vec3 vector = calculateInitialShotForce(rpm, pitch, yaw);
    printVec3(vector);
}