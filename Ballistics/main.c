#include <stdio.h>
#include "initialVelo.h"
#include "forces.h"
#include "Constants.h"

int main() {

    double rpm = 2000;
    double pitch = 85;
    double yaw = 0;

    printf("RPM: %f\n", rpm);
    printf("Pitch (In degrees): %f\n", pitch);
    printf("Yaw (In degrees): %f\n", yaw);

    Vec3 position = createVec3(0.0, 0.0, 0.0);
    Vec3 velocity = calculateInitialShotForce(rpm, pitch, yaw);

    double dt = 0.01; // timestep (seconds)
    double time = 0.0;

    while (position.z >= 0.0) {

        Vec3 force = returnForceVector(velocity, createVec3(0.0, 0, 0.0));

        Vec3 acceleration = scalarMultVec3(1.0 / ballMass, force);

        velocity = addVec3(velocity, scalarMultVec3(dt, acceleration));

        position = addVec3(position, scalarMultVec3(dt, velocity));

        time += dt;
    }

    printf("Final Posistion\n");
    printVec3(position);
    printf("Flight time: %.2fs\n", time);

    return 0;
}