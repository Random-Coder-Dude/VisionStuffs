package frc.robot.Graph;

import java.util.function.Supplier;

import com.pathplanner.lib.path.GoalEndState;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.pathfinding.Pathfinding;
import com.pathplanner.lib.trajectory.PathPlannerTrajectory;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.DriverStation;
import frc.robot.Constants;

public class Helpers {

    private static Supplier<Pose2d> robotPoseSupplier;
    private static Supplier<Double> timeLeftSupplier;

    public static void initialize(Supplier<Pose2d> robotPose, Supplier<Double> timeLeft) {
        robotPoseSupplier = robotPose;
        timeLeftSupplier = timeLeft;
    }

    public static double getPathTime(Pose2d targetPose) {
        // Pose2d robotPose = robotPoseSupplier.get();

        // Pathfinding.setStartPosition(robotPose.getTranslation());
        // Pathfinding.setGoalPosition(targetPose.getTranslation());

        // PathPlannerPath path = Pathfinding.getCurrentPath(
        //         Constants.PATH_CONSTRAINTS,
        //         new GoalEndState(0.0, targetPose.getRotation()));

        // if (!Pathfinding.isNewPathAvailable()) {
        //     return Double.POSITIVE_INFINITY;
        // } else if (path == null) {
        //     return Double.NEGATIVE_INFINITY;
        // }

        // PathPlannerTrajectory trajectory = path.generateTrajectory(
        //         new ChassisSpeeds(), //Change to robot Chassis speeds
        //         robotPose.getRotation(),
        //         Constants.ROBOT_CONFIG);

        // return trajectory.getTotalTimeSeconds();

        return 0.0;
    }

    public static double getRobotScore() {
        return 0.0;
    }

    public static double getMatchTime() {
        // double timeLeft = Math.max(0, DriverStation.getMatchTime());

        // if (DriverStation.isAutonomous()) {
        //     return timeLeft + 135;
        // } else {
        //     return timeLeft;
        // }
        return timeLeftSupplier.get();
    }

}
