#pragma once

/**
 * @file Constants.h
 * @brief Physical, mechanical, and optimizer constants for the FRC ball-shooter.
 *
 * All values are in SI units (meters, kilograms, seconds) unless otherwise noted.
 * Modify Constants.c to tune the model to a specific robot / game piece.
 */

/* ── Launcher mechanics ──────────────────────────────────────────────── */

/** @brief Diameter of the shooter flywheel wheel (meters). */
extern const double LAUNCHER_WHEEL_DIAMETER;

/** @brief Moment of inertia of the ball about its center (kg·m²). */
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

/** @brief Drag coefficient (Cd) for the ball. */
extern const double CD;

/** @brief Lift / Magnus coefficient (Cl). */
extern const double CL;

/* ── Shooter mounting offsets ────────────────────────────────────────── */

/** @brief Lateral offset of shooter exit from robot center (meters). */
extern const double SHOOTER_OFFSET_X;

/** @brief Forward offset of shooter exit from robot center (meters). */
extern const double SHOOTER_OFFSET_Y;

/** @brief Height of shooter exit above the floor (meters). */
extern const double SHOOTER_HEIGHT_OFFSET;

/* ── Simulation timesteps ────────────────────────────────────────────── */

/** @brief Fine timestep for full-precision simulation (seconds). */
extern const double SIM_DT_FINE;

/** @brief Coarse timestep used during optimization — ~10x faster (seconds). */
extern const double SIM_DT_COARSE;

/* ── Shooter parameter bounds ────────────────────────────────────────── */

/** @brief Minimum flywheel speed (rev/min). */
extern const double OPT_MIN_RPM;

/** @brief Maximum flywheel speed (rev/min). */
extern const double OPT_MAX_RPM;

/** @brief Minimum hood mechanism angle (degrees). */
extern const double OPT_MIN_HOOD;

/** @brief Maximum hood mechanism angle (degrees). */
extern const double OPT_MAX_HOOD;

/* ── Optimizer hyperparameters ───────────────────────────────────────── */

/** @brief Score below which the optimizer accepts a solution. */
extern const double OPT_SCORE_THRESHOLD;

/** @brief Maximum number of random restarts. */
extern const int    OPT_MAX_RESTARTS;

/** @brief Maximum gradient descent iterations per restart. */
extern const int    OPT_MAX_ITERATIONS;

/** @brief RPM finite-difference perturbation for gradient estimation. */
extern const double OPT_EPS_RPM;

/** @brief Hood angle finite-difference perturbation (degrees). */
extern const double OPT_EPS_HOOD;

/** @brief Turret angle finite-difference perturbation (degrees). */
extern const double OPT_EPS_TURRET;

/** @brief Initial learning rate. */
extern const double OPT_LR_INIT;

/** @brief Per-RPM learning rate scale (RPM units per normalized step). */
extern const double OPT_LR_SCALE_RPM;

/** @brief Per-hood learning rate scale (degrees per normalized step). */
extern const double OPT_LR_SCALE_HOOD;

/** @brief Per-turret learning rate scale (degrees per normalized step). */
extern const double OPT_LR_SCALE_TURRET;

/** @brief Learning rate shrink factor on failed line search step. */
extern const double OPT_LR_DECAY;

/** @brief Learning rate growth factor on successful step. */
extern const double OPT_LR_GROW;

/** @brief Minimum learning rate floor. */
extern const double OPT_LR_MIN;

/** @brief Maximum learning rate ceiling. */
extern const double OPT_LR_MAX;

/** @brief Half-width of random turret jitter on restarts (degrees). */
extern const double OPT_TURRET_JITTER;

/* ── Scoring weights ─────────────────────────────────────────────────── */

/** @brief Weight on XY landing error (position cost). */
extern const double SCORE_WEIGHT_POSITION;

/** @brief Weight on RPM sensitivity (robustness cost). */
extern const double SCORE_WEIGHT_SENSITIVITY;

/** @brief Weight on apex height excess above goalZ. */
extern const double SCORE_WEIGHT_APEX;

/** @brief RPM step used to estimate landing sensitivity. */
extern const double SCORE_RPM_SENSITIVITY_STEP;

/** @brief Height offset below goalZ used in non-scoreable penalty (meters).
 *  Shifts the dz term so the gradient always points upward. */
extern const double SCORE_UNSCOREABLE_DZ_OFFSET;

/** @brief Weight on dz² term in non-scoreable penalty. */
extern const double SCORE_UNSCOREABLE_DZ_WEIGHT;

/* ── Look-up tables ──────────────────────────────────────────────────── */

/** @brief Entry mapping flywheel RPM to ball exit speed. */
typedef struct {
    double rpm;
    double velocity;
} ShooterLUTEntry;

/** @brief Entry mapping hood mechanism angle to launch pitch. */
typedef struct {
    double hoodAngle;
    double pitch;
} HoodLUTEntry;

extern const ShooterLUTEntry SHOOTER_LUT[];
extern const int              SHOOTER_LUT_SIZE;

extern const HoodLUTEntry    HOOD_LUT[];
extern const int              HOOD_LUT_SIZE;
