#include <math.h>
#include "scoreTrajectory.h"
#include "Constants.h"

double scoreTrajectory(SimResult trajectory, Vec3 goalPose, ChassisSpeeds robot) {

    if (!trajectory.valid || !trajectory.comingFromTop) {
        return 1e9;
    }

    // --- Position error (m) ---
    Vec3   diff     = addVec3(trajectory.finalPosition, scalarMultVec3(-1.0, goalPose));
    double posError = magnitudeVec3(diff);

    // --- Height penalty: prefer lower arcs ---
    double heightPenalty = trajectory.maxHeight;

    // --- RPM sensitivity (m / RPM) ---
    const double rpmStep = 50.0;

    SimResult perturbed = calculateTrajectory(
        trajectory.RPM + rpmStep,
        trajectory.HoodAngle,
        trajectory.TurretAngle,
        goalPose.z,
        robot,
        trajectory.spin
    );

    double rpmSensitivity;
    if (!perturbed.valid) {
        rpmSensitivity = 1.0;  // treat infeasible perturbation as very sensitive
    } else {
        Vec3 deltaPos  = addVec3(perturbed.finalPosition,
                                 scalarMultVec3(-1.0, trajectory.finalPosition));
        rpmSensitivity = magnitudeVec3(deltaPos) / rpmStep;
    }

    // Weights tuned so all three terms are on the same order of magnitude
    // near a good solution:
    //   posError       : target 0 m,   bad ~1-5 m     -> x100 gives 0-2500
    //   excessHeight   : only penalise arc above goal  -> x0.1 keeps it minor
    //   rpmSensitivity : target ~0.001 m/RPM           -> x10 keeps it minor
    const double positionWeight    = 100.0;
    const double heightWeight      =   0.1;
    const double sensitivityWeight =  10.0;

    // Only penalise height above the goal — some arc is unavoidable and fine
    double excessHeight = trajectory.maxHeight - goalPose.z;
    if (excessHeight < 0.0) excessHeight = 0.0;

    return positionWeight    * posError      * posError      +
           heightWeight      * excessHeight  * excessHeight  +
           sensitivityWeight * rpmSensitivity * rpmSensitivity;
}