#pragma once

/**
 * @file forces.h
 * @brief Aerodynamic and gravitational force calculations acting on the ball.
 *
 * Three forces are modelled:
 *   1. Gravity      — constant downward force (F = m·g)
 *   2. Aerodynamic drag — opposes motion, scales with v²
 *   3. Magnus effect    — lift force due to backspin, perpendicular to v and ω
 *
 * All forces are returned as Vec3 values in Newtons.
 */

#include "structs.h"

/**
 * @brief Compute the gravitational force on the ball.
 *
 * Returns a downward force vector: F_gravity = (0, 0, m·g)
 * where g is negative (GRAVITY constant).
 *
 * @return Gravitational force vector (N).
 */
Vec3 calculateGravity(void);

/**
 * @brief Compute aerodynamic drag opposing the ball's velocity.
 *
 * Uses the standard drag equation:
 *   F_drag = -0.5 * ρ * v² * Cd * A * v̂
 *
 * where:
 *   ρ  = air density  (AIR_DENSITY, kg/m³)
 *   v  = speed        (m/s)
 *   Cd = drag coefficient (CD)
 *   A  = cross-sectional area of ball (π·r²)
 *   v̂  = unit vector in direction of velocity
 *
 * The negative sign ensures drag always opposes motion.
 *
 * @param velocity Current ball velocity vector (m/s).
 * @return Drag force vector (N), opposing velocity.
 */
Vec3 calculateDrag(Vec3 velocity);

/**
 * @brief Compute the Magnus (gyroscopic lift) force on a spinning ball.
 *
 * The Magnus effect causes a spinning ball to curve. The force is:
 *   F_magnus = 0.5 * ρ * Cl * A * r * (ω × v)
 *
 * where:
 *   ρ  = air density       (kg/m³)
 *   Cl = lift coefficient  (CL)
 *   A  = cross-sectional area (m²)
 *   r  = ball radius          (m)
 *   ω  = angular velocity vector of ball (rad/s)
 *   v  = translational velocity vector   (m/s)
 *   ×  = cross product (right-hand rule)
 *
 * For a shooter with backspin (ω pointing in -Y), the Magnus force adds
 * upward lift, which extends range and flattens the arc.
 *
 * @param velocity Ball translational velocity (m/s).
 * @param spin     Ball angular velocity vector (rad/s); direction = spin axis.
 * @return Magnus force vector (N).
 */
Vec3 calculateMagnus(Vec3 velocity, Vec3 spin);

/**
 * @brief Sum all forces acting on the ball at a given instant.
 *
 * Combines gravity, drag, and Magnus into a single net force vector:
 *   F_net = F_gravity + F_drag + F_magnus
 *
 * This vector is divided by ball mass in the integrator to get acceleration.
 *
 * @param currentVelocity Current ball velocity (m/s).
 * @param spinVector      Current ball angular velocity (rad/s).
 * @return Net force vector acting on the ball (N).
 */
Vec3 returnForceVector(Vec3 currentVelocity, Vec3 spinVector);
