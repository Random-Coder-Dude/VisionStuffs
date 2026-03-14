/**
 * @file generateTable.c
 * @brief Precomputes the shoot-on-the-move lookup table and writes it as JSON.
 *
 * Sweeps three independent dimensions:
 *   distance  — robot-to-goal range (meters)
 *   vx        — robot forward  velocity (m/s, robot-relative)
 *   vy        — robot lateral  velocity (m/s, robot-relative)
 *
 * Compile with OpenMP:
 *   gcc -O2 -fopenmp -o generateTable ^
 *       generateTable.c Constants.c structs.c forces.c ^
 *       forwardPass.c initialVelo.c optimize.c scoreTrajectory.c -lm
 *
 * Run:
 *   generateTable.exe > shooter_table.json
 */

#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include <omp.h>
#include <windows.h>
#include "optimize.h"
#include "Constants.h"

/* Ultra 7 265: 8 P-cores are logical processors 0-7 (no hyperthreading on Arrow Lake).
 * Affinity mask 0xFF = binary 11111111 = cores 0-7 only. */
#define PCORE_COUNT    8
#define PCORE_AFFINITY 0xFF

static void pin_to_pcores(void) {
    /* Called once per OpenMP thread at startup.
     * Each thread gets pinned to its own P-core so no two threads share a core
     * and no thread ever migrates to an E-core. */
    int tid = omp_get_thread_num();
    DWORD_PTR mask = (DWORD_PTR)1 << (tid % PCORE_COUNT);
    SetThreadAffinityMask(GetCurrentThread(), mask);
}

/* ── Table sweep parameters ──────────────────────────────────────────── */

#define GOAL_Z       1.8288

#define DIST_MIN     0.1
#define DIST_MAX     15.0
#define DIST_STEP    0.5

#define VX_MIN      -5.0
#define VX_MAX       5.0
#define VX_STEP      0.05

#define VY_MIN      -5.0
#define VY_MAX       5.0
#define VY_STEP      0.05

/* ── Helpers ─────────────────────────────────────────────────────────── */

static int countSteps(double min, double max, double step) {
    return (int)round((max - min) / step) + 1;
}

int main(void) {
    int distCount = countSteps(DIST_MIN, DIST_MAX, DIST_STEP);
    int vxCount   = countSteps(VX_MIN,   VX_MAX,   VX_STEP);
    int vyCount   = countSteps(VY_MIN,   VY_MAX,   VY_STEP);
    int total     = distCount * vxCount * vyCount;
    int sliceSize = vxCount * vyCount;

    fprintf(stderr, "Generating %d cells (%d dist x %d vx x %d vy) on %d threads\n",
            total, distCount, vxCount, vyCount, omp_get_max_threads());

    double *rpmArr    = (double *)malloc(total * sizeof(double));
    double *hoodArr   = (double *)malloc(total * sizeof(double));
    double *turretArr = (double *)malloc(total * sizeof(double));
    int    *validArr  = (int    *)malloc(total * sizeof(int));

    /* Warm-start seeds for the current distance slice.
     * Seeded from the previous distance slice's results — adjacent distances
     * have nearly identical solutions so the optimizer converges in very few
     * restarts instead of grinding all 200 from cold. */
    double *wsSeedRPM  = (double *)malloc(sliceSize * sizeof(double));
    double *wsSeedHood = (double *)malloc(sliceSize * sizeof(double));
    for (int i = 0; i < sliceSize; i++) {
        wsSeedRPM [i] = 3000.0;
        wsSeedHood[i] = 15.0;
    }

    int done = 0; /* atomic counter for progress, written under critical */

    /* ── Outer loop over distance slices — sequential ──────────────────
     * Must be sequential so each slice can warm-start from the previous one.
     * The inner (vx, vy) loop is fully parallel — all cells in a slice are
     * independent of each other. */
    for (int di = 0; di < distCount; di++) {
        double dist = DIST_MIN + di * DIST_STEP;

        /* Snapshot warm-start seeds for this slice (read-only inside parallel) */
        double *sliceSeedRPM  = (double *)malloc(sliceSize * sizeof(double));
        double *sliceSeedHood = (double *)malloc(sliceSize * sizeof(double));
        for (int i = 0; i < sliceSize; i++) {
            sliceSeedRPM [i] = wsSeedRPM [i];
            sliceSeedHood[i] = wsSeedHood[i];
        }

        #pragma omp parallel num_threads(PCORE_COUNT)
        {
            pin_to_pcores();  /* pin this thread to its assigned P-core */
            #pragma omp for schedule(dynamic, 1) collapse(2)
            for (int xi = 0; xi < vxCount; xi++) {
            for (int yi = 0; yi < vyCount; yi++) {
                double vx = VX_MIN + xi * VX_STEP;
                double vy = VY_MIN + yi * VY_STEP;
                int    si = xi * vyCount + yi;   /* index within this slice  */
                int   idx = di * sliceSize + si; /* index in full flat array */

                OptimizeResult res = optimize(
                    sliceSeedRPM [si],  /* warm-started from prev distance slice */
                    sliceSeedHood[si],
                    vx, vy, 0.0,
                    dist, 0.0, GOAL_Z
                );

                rpmArr   [idx] = res.rpm;
                hoodArr  [idx] = res.hoodAngle;
                turretArr[idx] = res.turretAngle;
                validArr [idx] = (res.score <= OPT_SCORE_THRESHOLD) ? 1 : 0;

                #pragma omp critical
                {
                    done++;
                    if (done % 100 == 0 || done == total) {
                        fprintf(stderr, "  [%d / %d]  dist=%.2f vx=%.1f vy=%.1f  "
                                "rpm=%.0f hood=%.1f turret=%.2f  score=%.3f  valid=%d\n",
                                done, total, dist, vx, vy,
                                res.rpm, res.hoodAngle, res.turretAngle,
                                res.score, validArr[idx]);
                    }
                }
            }
            } /* end omp for */
        } /* end omp parallel */

        /* Update warm-start seeds for next distance slice.
         * Only propagate from valid cells — invalid ones keep the previous seed
         * so we don't poison the warm start with a garbage solution. */
        for (int si = 0; si < sliceSize; si++) {
            int idx = di * sliceSize + si;
            if (validArr[idx]) {
                wsSeedRPM [si] = rpmArr [idx];
                wsSeedHood[si] = hoodArr[idx];
            }
        }

        free(sliceSeedRPM);
        free(sliceSeedHood);
    }

    /* ── Write JSON ──────────────────────────────────────────────────── */

    printf("{\n");
    printf("  \"meta\": {\n");
    printf("    \"goalZ\":     %.4f,\n", GOAL_Z);
    printf("    \"distMin\":   %.4f, \"distMax\":  %.4f, \"distStep\": %.4f, \"distCount\": %d,\n",
           DIST_MIN, DIST_MAX, DIST_STEP, distCount);
    printf("    \"vxMin\":     %.4f, \"vxMax\":    %.4f, \"vxStep\":   %.4f, \"vxCount\":   %d,\n",
           VX_MIN, VX_MAX, VX_STEP, vxCount);
    printf("    \"vyMin\":     %.4f, \"vyMax\":    %.4f, \"vyStep\":   %.4f, \"vyCount\":   %d\n",
           VY_MIN, VY_MAX, VY_STEP, vyCount);
    printf("  },\n");

    printf("  \"rpm\": [");
    for (int i = 0; i < total; i++) printf("%.2f%s", rpmArr[i],    i < total-1 ? "," : "");
    printf("],\n");

    printf("  \"hood\": [");
    for (int i = 0; i < total; i++) printf("%.4f%s", hoodArr[i],   i < total-1 ? "," : "");
    printf("],\n");

    printf("  \"turret\": [");
    for (int i = 0; i < total; i++) printf("%.4f%s", turretArr[i], i < total-1 ? "," : "");
    printf("],\n");

    printf("  \"valid\": [");
    for (int i = 0; i < total; i++) printf("%d%s",   validArr[i],  i < total-1 ? "," : "");
    printf("]\n}\n");

    free(rpmArr); free(hoodArr); free(turretArr); free(validArr);
    free(wsSeedRPM); free(wsSeedHood);

    fprintf(stderr, "\nDone. %d entries written.\n", total);
    return 0;
}
