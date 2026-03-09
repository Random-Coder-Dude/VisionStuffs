/**
 * @file Constants.c
 * @brief Definitions of all physical and mechanical constants.
 *
 * Edit these values to match your specific robot and game piece.
 * All values are in SI units (meters, kilograms, seconds) unless noted.
 */

#include "Constants.h"

/* ── Launcher mechanics ──────────────────────────────────────────────── */

/** 4-inch wheel converted to meters (4 * 0.0254 = 0.1016 m). */
const double LAUNCHER_WHEEL_DIAMETER = 4.0 * 0.0254;

/** Ball moment of inertia: (2/5) * 0.2268 * (0.075)^2 ≈ 0.000256 kg·m²
 *  — adjust if ball is not a uniform solid sphere. */
const double BALL_MOI = 0.0002552;

/* ── Projectile properties ───────────────────────────────────────────── */

/** Standard FRC 2022 cargo mass. */
const double BALL_MASS = 0.2268;

/** 6-inch diameter in meters. */
const double PROJECTILE_DIAMETER = 0.1524;

/* ── Environmental constants ─────────────────────────────────────────── */

/** Standard gravity, directed downward. */
const double GRAVITY = -9.81;

/** Sea-level air density at 15 °C. */
const double AIR_DENSITY = 1.225;

/* ── Aerodynamic coefficients ────────────────────────────────────────── */

/** Drag coefficient for a smooth sphere in turbulent flow. */
const double CD = 0.47;

/** Magnus / lift coefficient — tune empirically. */
const double CL = 0.3;

/* ── Shooter mounting offsets ────────────────────────────────────────── */

/** Shooter is centered laterally on the robot. */
const double SHOOTER_OFFSET_X = 0.0;

/** Shooter is centered fore-aft on the robot. */
const double SHOOTER_OFFSET_Y = 0.0;

/** Shooter exits the ball 0.5 m above the floor. */
const double SHOOTER_HEIGHT_OFFSET = 0.5;
