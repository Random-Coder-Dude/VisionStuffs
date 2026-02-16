package frc.robot.Graph;

import java.util.List;
import java.util.function.Supplier;

import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.path.PathPoint;
import com.pathplanner.lib.trajectory.PathPlannerTrajectory;

import edu.wpi.first.math.geometry.Pose2d;
import frc.robot.Constants;

public class Helpers {

    private static Supplier<Pose2d> robotPoseSupplier;

    public static void initialize(Supplier<Pose2d> robotPose) {
        robotPoseSupplier = robotPose;
    }

    public static double getPathTime(Pose2d targetPose) {
        Pose2d robotPose = robotPoseSupplier.get();
        PathPoint start = new PathPoint(robotPose.getTranslation());
        PathPoint end = new PathPoint(targetPose.getTranslation());
        PathPlannerPath path = PathPlannerPath.fromPathPoints(List.of(start, end), Constants.PATH_CONSTRAINTS, null);

        return path.getIdealTrajectory(robotPose.getRotation(), robotPose.getRotation()).getTotalTimeSeconds();
    }

    public static double getRobotScore() {
        Pose2d robotPose = robotPoseSupplier.get();
        return 0.0;
    }

    public static double getMatchTime() {
        return 0.0;
    }
}
