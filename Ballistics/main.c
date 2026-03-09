#include <stdio.h>
#include "optimize.h"
#include "forwardPass.h"
#include "structs.h"

int main(void) {
    double goalX = 3.47;
    double goalY = 0.0;
    double goalZ = 2.0;

    OptimizeResult result = optimize(
        /* initial guesses */
        4000.0,  /* rpm  */
        20.0,    /* hood */
        /* robot velocity */
        0.0,     /* robotVx    (m/s)   */
        0.0,     /* robotVy    (m/s)   */
        0.0,     /* robotOmega (rad/s) */
        /* goal — robot-relative */
        goalX, goalY, goalZ
    );

    /* Re-simulate at full precision for landing and max height. */
    ChassisSpeeds robot = createChassisSpeeds(0.0, 0.0, 0.0);
    SimResult best = calculateTrajectory(result.rpm, result.hoodAngle,
                                         result.turretAngle, goalZ, robot);

    printf("\n=== BEST SOLUTION ===\n");
    printf("Score      : %.6f\n",   result.score);
    printf("RPM        : %.2f\n",   result.rpm);
    printf("Hood       : %.2f deg\n", result.hoodAngle);
    printf("Turret     : %.2f deg\n", result.turretAngle);
    printf("Landing XY : (%.3f, %.3f) m  [goal: (%.2f, %.2f)]\n",
           best.finalPosition.x, best.finalPosition.y, goalX, goalY);
    printf("Max height : %.3f m  [goal z: %.2f]\n", best.maxHeight, goalZ);
    printf("Scoreable  : %s\n", best.scoreable ? "yes" : "NO");

    return 0;
}
