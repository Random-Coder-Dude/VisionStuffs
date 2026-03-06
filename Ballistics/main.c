#include <stdio.h>
#include "forwardPass.h"
#include "structs.h"
#include "Constants.h"

int main() {
    // Example robot velocity (vx, vy, omega)
    ChassisSpeeds robot = createChassisSpeeds(0, 0, 0);

    // Example shot parameters
    double rpm = 5000;          // shooter RPM
    double hoodAngle = 45.0;    // hood angle in degrees
    double turretAngle = 0.0;   // yaw in degrees
    double goalZ = 2.0;         // target height in meters

    // Calculate trajectory
    SimResult result = calculateTrajectory(rpm, hoodAngle, turretAngle, goalZ, robot);

    // Print results
    printf("Final Position: x=%.2f, y=%.2f, z=%.2f\n",
           result.finalPosition.x, result.finalPosition.y, result.finalPosition.z);
    printf("Flight Time: %.2fs\n", result.shotTime);
    printf("Max Height: %.2fm\n", result.maxHeight);
    printf("Coming from top: %s\n", result.comingFromTop ? "Yes" : "No");

    return 0;
}