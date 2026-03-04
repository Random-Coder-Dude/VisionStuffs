#include "forwardPass.h"
#include <stdio.h>

int main() {
    for (int z = 0; z < 10; z++) {
        printf("current rpm %d\n", 5000 + 100*z);
        simulationResult sim = simulateFinalPosistion(5000 + 100*z, 75, 0, 0);
        Vec3 posistion = sim.posistion;
        posistion = scalarMult(3.281, posistion);
        printVec3(posistion);
    }
    return 0;
}