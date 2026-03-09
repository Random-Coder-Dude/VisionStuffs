#pragma once

/**
 * @file Constants.h
 * @brief Physical and mechanical constants for the FRC ball-shooter simulation.
 *
 * All values are in SI units (meters, kilograms, seconds) unless otherwise noted.
 * Modify Constants.c to tune the model to a specific robot / game piece.
 */

/* ── Launcher mechanics ──────────────────────────────────────────────── */

/** @brief Diameter of the shooter flywheel wheel (meters). */
extern const double LAUNCHER_WHEEL_DIAMETER;

/** @brief Moment of inertia of the ball about its center (kg·m²).
 *
 *  For a uniform solid sphere: I = (2/5) * m * r²
 *  Used to compute spin angular velocity from shooter RPM via energy transfer.
 */
extern const double BALL_MOI;

/* ── Projectile properties ───────────────────────────────────────────── */

/** @brief Mass of the game piece / ball (kg). */
extern const double BALL_MASS;

/** @brief Outer diameter of the ball (meters). */
extern const double PROJECTILE_DIAMETER;

/* ── Environmental constants ─────────────────────────────────────────── */

/** @brief Gravitational acceleration (m/s², negative = downward). */
extern const double GRAVITY;

/** @brief Density of air at standard conditions (kg/m³). */
extern const double AIR_DENSITY;

/* ── Aerodynamic coefficients ────────────────────────────────────────── */

/** @brief Drag coefficient (Cd) for the ball.
 *
 *  A sphere in turbulent flow typically has Cd ≈ 0.47.
 *  Tune with empirical shot data if available.
 */
extern const double CD;

/** @brief Lift / Magnus coefficient (Cl).
 *
 *  Scales the Magnus cross-product force. Higher values = more curve.
 *  Tune with empirical shot data if available.
 */
extern const double CL;

/* ── Shooter mounting offsets ────────────────────────────────────────── */

/** @brief Lateral offset of shooter exit from robot center (meters). */
extern const double SHOOTER_OFFSET_X;

/** @brief Forward offset of shooter exit from robot center (meters). */
extern const double SHOOTER_OFFSET_Y;

/** @brief Height of shooter exit above the floor (meters). */
extern const double SHOOTER_HEIGHT_OFFSET;
