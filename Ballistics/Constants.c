/**
 * @file Constants.c
 * @brief Definitions of all physical, mechanical, and optimizer constants.
 *
 * Edit these values to match your specific robot, game piece, and tuning goals.
 * All values are in SI units (meters, kilograms, seconds) unless noted.
 */

#include "Constants.h"

/* ── Launcher mechanics ──────────────────────────────────────────────── */

/** 4-inch wheel converted to meters. */
const double LAUNCHER_WHEEL_DIAMETER = 4.0 * 0.0254;

/** Ball MOI: (2/5) * 0.2268 * (0.075)^2 */
const double BALL_MOI = 0.0002552;

/* ── Projectile properties ───────────────────────────────────────────── */

/** Standard FRC 2022 cargo mass. */
const double BALL_MASS = 0.2268;

/** 6-inch diameter in meters. */
const double PROJECTILE_DIAMETER = 0.15;

/* ── Environmental constants ─────────────────────────────────────────── */

/** Standard gravity, directed downward. */
const double GRAVITY = -9.81;

/** Sea-level air density at 15°C. */
const double AIR_DENSITY = 1.225;

/* ── Aerodynamic coefficients ────────────────────────────────────────── */

/** Drag coefficient for a smooth sphere in turbulent flow. */
const double CD = 0.47;

/** Magnus / lift coefficient — tune empirically. */
const double CL = 0.3;

/* ── Shooter mounting offsets ────────────────────────────────────────── */

const double SHOOTER_OFFSET_X      = 0.0;
const double SHOOTER_OFFSET_Y      = 0.0;
const double SHOOTER_HEIGHT_OFFSET = 0.5;

/* ── Simulation timesteps ────────────────────────────────────────────── */

/** Full-precision timestep (seconds). */
const double SIM_DT_FINE   = 0.001;

/** Coarse timestep for optimizer evaluations — 10x faster. */
const double SIM_DT_COARSE = 0.01;

/* ── Shooter parameter bounds ────────────────────────────────────────── */

const double OPT_MIN_RPM  =    0.0;
const double OPT_MAX_RPM  = 6000.0;
const double OPT_MIN_HOOD =    0.0;
const double OPT_MAX_HOOD =   30.0;

/* ── Optimizer hyperparameters ───────────────────────────────────────── */

const double OPT_SCORE_THRESHOLD =   10.0;
const int    OPT_MAX_RESTARTS    = 200;
const int    OPT_MAX_ITERATIONS  = 200;

const double OPT_EPS_RPM    = 50.0;
const double OPT_EPS_HOOD   =  0.5;
const double OPT_EPS_TURRET =  0.5;

const double OPT_LR_INIT         =   5.0;
const double OPT_LR_SCALE_RPM    = 100.0;
const double OPT_LR_SCALE_HOOD   =   1.0;
const double OPT_LR_SCALE_TURRET =   1.0;
const double OPT_LR_DECAY        =   0.5;
const double OPT_LR_GROW         =   1.05;
const double OPT_LR_MIN          =   1e-4;
const double OPT_LR_MAX          = 200.0;

const double OPT_TURRET_JITTER = 45.0;

/* ── Scoring weights ─────────────────────────────────────────────────── */

const double SCORE_WEIGHT_POSITION    = 100.0;
const double SCORE_WEIGHT_SENSITIVITY =   5.0;
const double SCORE_WEIGHT_APEX        =   0.0;
/* BUG FIX #2: This step can be negative (perturbing RPM downward is fine),
 * but scoreTrajectory.c now uses fabs() when dividing, so the sign no longer
 * corrupts the normalization. Left negative to test robustness to lower RPM. */
const double SCORE_RPM_SENSITIVITY_STEP = -50.0;
const double SCORE_UNSCOREABLE_DZ_OFFSET =  0.5;
const double SCORE_UNSCOREABLE_DZ_WEIGHT = 10.0;

/* ── Look-up tables ──────────────────────────────────────────────────── */

/** RPM → ball exit speed. Calibrate empirically on your shooter rig. */
const ShooterLUTEntry SHOOTER_LUT[] = {
    {1000.0,  3.5},
    {2000.0,  7.0},
    {3000.0, 10.2},
    {4000.0, 13.0},
    {5000.0, 15.5},
    {5500.0, 16.8},
    {6000.0, 17.5}
};
const int SHOOTER_LUT_SIZE = (int)(sizeof(SHOOTER_LUT) / sizeof(SHOOTER_LUT[0]));

/* BUG FIX #3: Fixed comment — 0° hood = 0° pitch (flat/horizontal shot);
 * 30° hood = 60° pitch (steep arc). The old comment claimed the opposite
 * (0° = ~85° vertical, 30° = ~10° flat) which directly contradicted the table. */
const HoodLUTEntry HOOD_LUT[] = {
    { 0.0,  0.0},
    { 2.0,  4.0},
    { 4.0,  8.0},
    { 6.0, 12.0},
    { 8.0, 16.0},
    {10.0, 20.0},
    {12.0, 24.0},
    {14.0, 28.0},
    {16.0, 32.0},
    {18.0, 36.0},
    {20.0, 40.0},
    {22.0, 44.0},
    {24.0, 48.0},
    {26.0, 52.0},
    {28.0, 56.0},
    {30.0, 60.0}
};
const int HOOD_LUT_SIZE = (int)(sizeof(HOOD_LUT) / sizeof(HOOD_LUT[0]));
