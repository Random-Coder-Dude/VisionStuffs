#pragma once

/**
 * @file scoreTrajectory.h
 * @brief Continuous scoring function for gradient-descent trajectory optimization.
 *
 * The scorer converts a SimResult into a single scalar cost that the optimizer
 * in main.c minimizes. Lower scores are better; a score of 0 would mean a
 * perfect shot with zero position error, minimum arc height, and zero RPM
 * sensitivity.
 *
 * Three components contribute to the score:
 *
 *   1. **Position error** — Euclidean distance (XY plane) between where the
 *      ball lands at goalZ and the desired (goalPose.x, goalPose.y).
 *      Weighted heavily since hitting the target is the primary objective.
 *
 *   2. **Height penalty** — Excess arc height above goalZ. Prefer lower arcs
 *      that are less sensitive to wind and easier to control mechanically.
 *
 *   3. **RPM sensitivity** — How much the landing position shifts per unit
 *      change in flywheel RPM. Lower sensitivity = more repeatable shots.
 *      Normalized via a sigmoid-like mapping to prevent extreme outliers from
 *      dominating the gradient.
 *
 *   4. **Hard penalties** — Large additive constants for physically invalid
 *      trajectories (ball never reaches goalZ) or trajectories that do not
 *      arrive from above the goal (flat shots that graze the rim).
 */

#include "forwardPass.h"
#include "structs.h"

/**
 * @brief Compute the optimization cost for a simulated trajectory.
 *
 * The returned score is always non-negative and continuous, which is required
 * for gradient-descent to work correctly. Hard penalties use large-but-finite
 * values (not infinity or NaN) so gradients still flow toward valid solutions
 * even from an invalid starting point.
 *
 * @param trajectory  Simulation result from calculateTrajectory().
 * @param goalPose    Target position: (x, y) = desired landing point (m),
 *                                     z       = target height (m).
 * @param robot       Robot velocity at the time of the shot.  Forwarded to
 *                    the internal sensitivity simulation.
 * @return Non-negative scalar cost. Lower = better shot solution.
 */
double scoreTrajectory(SimResult trajectory, Vec3 goalPose, ChassisSpeeds robot);
