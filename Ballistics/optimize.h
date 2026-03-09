#pragma once

/**
 * @file optimize.h
 * @brief Public API for the shooter parameter optimizer.
 */

typedef struct {
    double rpm;
    double hoodAngle;
    double turretAngle;
    double score;
} OptimizeResult;

/**
 * @brief Find the optimal RPM, hood angle, and turret angle to hit a goal.
 *
 * @param rpm          Initial RPM guess (rev/min).
 * @param hood         Initial hood angle guess (degrees).
 * @param turret       Initial turret angle guess (degrees) — ignored in favor of atan2.
 * @param robotX       Robot field-relative X position (meters).
 * @param robotY       Robot field-relative Y position (meters).
 * @param robotHeading Robot heading in field frame (radians, CCW+).
 * @param robotVx      Robot translational velocity X (m/s).
 * @param robotVy      Robot translational velocity Y (m/s).
 * @param robotOmega   Robot rotation rate (rad/s, CCW+). Used to compute tangential
 *                     velocity at the shooter mount offset from robot center.
 * @param goalX        Goal field-relative X position (meters).
 * @param goalY        Goal field-relative Y position (meters).
 * @param goalZ        Goal height (meters).
 * @return OptimizeResult with best rpm, hoodAngle, turretAngle, and score.
 */
OptimizeResult optimize(double rpm, double hood,
                        double robotVx, double robotVy, double robotOmega,
                        double goalX, double goalY, double goalZ);
