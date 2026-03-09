/**
 * @file initialVelo.c
 * @brief Look-up-table based initial velocity and spin calculation.
 *
 * Two LUTs map hardware settings to physical quantities:
 *   shooterLUT — RPM → ball exit speed (m/s)
 *   hoodLUT    — hood mechanism angle → geometric launch pitch (degrees)
 *
 * Both LUTs are linearly interpolated between entries. Values outside the
 * table bounds are clamped to the nearest endpoint.
 *
 * Spin is derived from flywheel surface speed:
 *   omega = (RPM * 2π/60) / r_wheel   [rad/s, about the ball's lateral axis]
 * The direction is set to -Y (backspin when shooting +X) which produces
 * upward Magnus lift via the cross product (omega × v).
 */

#include <math.h>
#include "initialVelo.h"
#include "Constants.h"

/* =========================================================================
 * Look-up Tables
 * ========================================================================= */

/**
 * @brief Maps flywheel RPM to ball exit speed (m/s).
 *
 * Entries must be sorted in ascending RPM order for the interpolation search
 * to work correctly. Calibrate these values empirically on your shooter rig.
 */
typedef struct {
    double rpm;      /**< Flywheel speed (rev/min)  */
    double velocity; /**< Ball exit speed   (m/s)   */
} ShooterEntry;

static const ShooterEntry SHOOTER_LUT[] = {
    {1000.0,  3.5},
    {2000.0,  7.0},
    {3000.0, 10.2},
    {4000.0, 13.0},
    {5000.0, 15.5},
    {5500.0, 16.8},
    {6000.0, 17.5}
};
static const int SHOOTER_LUT_SIZE = (int)(sizeof(SHOOTER_LUT) / sizeof(SHOOTER_LUT[0]));

/**
 * @brief Maps the hood mechanism angle to the geometric launch pitch (degrees).
 *
 * A hood angle of 0° produces a nearly vertical (~85°) shot; 30° gives a
 * flatter (~10°) trajectory. Entries must be in ascending hood-angle order.
 */
typedef struct {
    double hoodAngle; /**< Hood mechanism angle (degrees) */
    double pitch;     /**< Resulting launch pitch (degrees above horizontal) */
} HoodAngleEntry;

static const HoodAngleEntry HOOD_LUT[] = {
    { 0.0, 85.0},
    { 2.0, 80.0},
    { 4.0, 75.0},
    { 6.0, 70.0},
    { 8.0, 65.0},
    {10.0, 60.0},
    {12.0, 55.0},
    {14.0, 50.0},
    {16.0, 45.0},
    {18.0, 40.0},
    {20.0, 35.0},
    {22.0, 30.0},
    {24.0, 25.0},
    {26.0, 20.0},
    {28.0, 15.0},
    {30.0, 10.0}
};
static const int HOOD_LUT_SIZE = (int)(sizeof(HOOD_LUT) / sizeof(HOOD_LUT[0]));

/* =========================================================================
 * Private Helpers
 * ========================================================================= */

/**
 * @brief Linearly interpolate the launch pitch from the hood mechanism angle.
 *
 * Scans the HOOD_LUT for the bracketing entries and interpolates.
 * Clamps to the first/last entry if hoodAngle is out of range.
 *
 * @param hoodAngle Hood mechanism angle (degrees).
 * @return Geometric launch pitch (degrees above horizontal).
 */
static double getPitchFromHood(double hoodAngle) {
    /* Below table minimum — clamp to first entry. */
    if (hoodAngle <= HOOD_LUT[0].hoodAngle) return HOOD_LUT[0].pitch;

    /* Above table maximum — clamp to last entry. */
    if (hoodAngle >= HOOD_LUT[HOOD_LUT_SIZE - 1].hoodAngle)
        return HOOD_LUT[HOOD_LUT_SIZE - 1].pitch;

    /* Find bracketing segment and linearly interpolate. */
    for (int i = 0; i < HOOD_LUT_SIZE - 1; i++) {
        if (hoodAngle >= HOOD_LUT[i].hoodAngle &&
            hoodAngle <= HOOD_LUT[i + 1].hoodAngle)
        {
            double t = (hoodAngle          - HOOD_LUT[i].hoodAngle) /
                       (HOOD_LUT[i+1].hoodAngle - HOOD_LUT[i].hoodAngle);
            return HOOD_LUT[i].pitch + t * (HOOD_LUT[i+1].pitch - HOOD_LUT[i].pitch);
        }
    }

    return HOOD_LUT[HOOD_LUT_SIZE - 1].pitch; /* Should be unreachable. */
}

/**
 * @brief Linearly interpolate ball exit speed from flywheel RPM.
 *
 * Scans the SHOOTER_LUT for the bracketing entries and interpolates.
 * Clamps to the first/last entry if rpm is out of range.
 *
 * @param rpm Flywheel speed (rev/min).
 * @return Ball exit speed (m/s).
 */
static double getVelocityFromRPM(double rpm) {
    if (rpm <= SHOOTER_LUT[0].rpm) return SHOOTER_LUT[0].velocity;
    if (rpm >= SHOOTER_LUT[SHOOTER_LUT_SIZE - 1].rpm)
        return SHOOTER_LUT[SHOOTER_LUT_SIZE - 1].velocity;

    for (int i = 0; i < SHOOTER_LUT_SIZE - 1; i++) {
        if (rpm >= SHOOTER_LUT[i].rpm && rpm <= SHOOTER_LUT[i + 1].rpm) {
            double t = (rpm              - SHOOTER_LUT[i].rpm) /
                       (SHOOTER_LUT[i+1].rpm - SHOOTER_LUT[i].rpm);
            return SHOOTER_LUT[i].velocity
                   + t * (SHOOTER_LUT[i+1].velocity - SHOOTER_LUT[i].velocity);
        }
    }

    return SHOOTER_LUT[SHOOTER_LUT_SIZE - 1].velocity; /* Should be unreachable. */
}

/* =========================================================================
 * Public API
 * ========================================================================= */

/**
 * @brief Compute the initial ball velocity vector and spin from shooter params.
 *
 * Coordinate convention (field-relative, robot shooter looking +X):
 *   pitch — elevation angle above the XY plane
 *   yaw   — rotation in the XY plane (turret angle)
 *
 * Ball direction unit vector:
 *   vx = cos(pitch) * cos(yaw)
 *   vy = cos(pitch) * sin(yaw)
 *   vz = sin(pitch)
 *
 * Robot-motion corrections applied after scaling by exit speed:
 *   1. Translational: add (robot.vx, robot.vy, 0)
 *   2. Rotational:    add omega × r_offset, where r_offset is the
 *      shooter's position relative to the robot rotation center.
 *      This gives the tangential velocity of the shooter exit point.
 *
 * Spin is derived from flywheel surface speed and directed as backspin
 * (omega points in -Y when shooting +X) so Magnus lift pushes the ball up.
 */
Vec3 calculateInitialVelocity(double RPM, double hoodAngle, double turretAngle,
                              ChassisSpeeds robot, Vec3 *spinOut)
{
    /* --- Step 1 & 2: resolve hardware angles to physical quantities --- */
    double pitch = getPitchFromHood(hoodAngle) * M_PI / 180.0; /* radians */
    double yaw   = turretAngle                 * M_PI / 180.0; /* radians */

    double exitSpeed = getVelocityFromRPM(RPM);

    /* --- Step 3 & 4: build shooter-frame velocity vector --- */
    Vec3 ballVelo = createVec3(
        cos(pitch) * cos(yaw),
        cos(pitch) * sin(yaw),
        sin(pitch)
    );
    ballVelo = scalarMultVec3(exitSpeed, ballVelo);

    /* --- Step 5: add robot translational velocity --- */
    Vec3 robotTransVelo = createVec3(robot.vx, robot.vy, 0.0);
    ballVelo = addVec3(ballVelo, robotTransVelo);

    /* --- Step 6: add tangential velocity from robot rotation ---
     *
     * If the shooter exit is offset from the robot's rotation center by
     * (SHOOTER_OFFSET_X, SHOOTER_OFFSET_Y), then at yaw-rate omega the
     * shooter tip has tangential velocity:
     *   v_tip_x = -omega * SHOOTER_OFFSET_Y
     *   v_tip_y =  omega * SHOOTER_OFFSET_X
     * (cross product of ω·ẑ with the offset vector)
     */
    Vec3 robotRotVelo = createVec3(
        -robot.omega * SHOOTER_OFFSET_Y,
         robot.omega * SHOOTER_OFFSET_X,
         0.0
    );
    ballVelo = addVec3(ballVelo, robotRotVelo);

    /* --- Step 7: compute ball spin from flywheel surface speed ---
     *
     * The flywheel imparts backspin. Surface speed of the wheel:
     *   v_surface = (RPM * 2π/60) * (LAUNCHER_WHEEL_DIAMETER / 2)
     *
     * Ball angular velocity (assuming 100% spin transfer, no slip):
     *   omega_ball = v_surface / r_ball
     *
     * Direction: -Y for backspin when shooting in the +X direction.
     * This cross-product (ω × v) with v = +X and ω = -Y gives +Z (upward),
     * providing the Magnus lift that helps the ball arc over the goal.
     */
    if (spinOut != NULL) {
        double wheelRadius = LAUNCHER_WHEEL_DIAMETER / 2.0;
        double ballRadius  = PROJECTILE_DIAMETER     / 2.0;
        double wheelSurface = (RPM * 2.0 * M_PI / 60.0) * wheelRadius;
        double spinMag      = wheelSurface / ballRadius;

        /* Backspin: rotate the -Y axis by the turret yaw so spin is
           always perpendicular to the shot direction in the horizontal plane. */
        spinOut->x =  sin(yaw) * spinMag;
        spinOut->y = -cos(yaw) * spinMag;
        spinOut->z =  0.0;
    }

    return ballVelo;
}
