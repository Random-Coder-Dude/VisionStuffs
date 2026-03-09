#include <stdio.h>
#include "optimize.h"

int main(void) {
    OptimizeResult result = optimize(
        /* initial guesses */
        4000.0,  /* rpm  */
        20.0,    /* hood */
        /* robot velocity */
        2.0,     /* robotVx    (m/s)   */
        2.0,     /* robotVy    (m/s)   */
        1.0,     /* robotOmega (rad/s) */
        /* goal — robot-relative */
        3.47,    /* goalX (meters) */
        0.0,     /* goalY (meters) */
        2.0      /* goalZ (meters) */
    );

    printf("\n=== BEST SOLUTION ===\n");
    printf("Score  : %.6f\n",     result.score);
    printf("RPM    : %.2f\n",     result.rpm);
    printf("Hood   : %.2f deg\n", result.hoodAngle);
    printf("Turret : %.2f deg\n", result.turretAngle);

    return 0;
}
