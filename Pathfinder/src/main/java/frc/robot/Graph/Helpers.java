package frc.robot.Graph;

import java.util.function.Supplier;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.DriverStation;


public class Helpers {

    private static Supplier<Pose2d> robotPoseSupplier;

    public static void initialize(Supplier<Pose2d> robotPose) {
        robotPoseSupplier = robotPose;
    }

    public static double getPathTime(Pose2d targetPose) {
        Pose2d robotPose = robotPoseSupplier.get();

        return 0.0;
    }

    public static double getRobotScore() {
        return 0.0;
    }

    public static double getMatchTime() {
        double timeLeft = Math.max(0, DriverStation.getMatchTime());

        if (DriverStation.isAutonomous()) {
            return timeLeft + 135;
        } else {
            return timeLeft;
        }
    }

}
