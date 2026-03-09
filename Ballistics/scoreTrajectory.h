#pragma once

/**
 * @file scoreTrajectory.h
 * @brief Scoring function for gradient-descent trajectory optimization.
 *
 * Uses trajectory.scoreable (the single validity gate from forwardPass) to
 * choose between two cost formulations:
 *
 *   scoreable = true  → 2-D XY miss distance + RPM sensitivity penalty.
 *   scoreable = false → 3-D distance to goal point (smooth gradient toward
 *                       valid trajectories, no hard penalty cliff).
 */

#include "forwardPass.h"
#include "structs.h"

/**
 * @brief Compute the optimization cost for a simulated trajectory.
 *
 * @param trajectory  Result from calculateTrajectory().
 * @param goalPose    (x, y) = desired landing XY (m), z = target height (m).
 * @param robot       Robot velocity forwarded to the sensitivity simulation.
 * @return Non-negative scalar cost. Lower = better.
 */
double scoreTrajectory(SimResult trajectory, Vec3 goalPose, ChassisSpeeds robot);
