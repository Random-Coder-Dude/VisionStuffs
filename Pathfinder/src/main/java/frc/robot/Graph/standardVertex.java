package frc.robot.Graph;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.Command;

public class standardVertex implements iVertex {

    private double pointAdjustPeak;
    private double pointAdjustStd;
    private double pointAdjustScalar;
    private Command runCommand;
    private Pose2d targetPose;
    private double expectedPoints;
    private double expectedRP;
    private String name;

    public standardVertex(double pointAdjustPeak, double pointAdjustStd, double pointAdjustScalar, Command runCommand,
            Pose2d targetPose,
            double expectedPoints, double expectedRP, String name) {
        this.pointAdjustPeak = pointAdjustPeak;
        this.pointAdjustStd = pointAdjustStd;
        this.pointAdjustScalar = pointAdjustScalar;
        this.runCommand = runCommand;
        this.targetPose = targetPose;
        this.expectedPoints = expectedPoints;
        this.expectedRP = expectedRP;
        this.name = name;
    }

    public double pointAdjust(double timeLeftInMatch) {
        double timeDiff = timeLeftInMatch - pointAdjustPeak;
        double exponent = -Math.pow(timeDiff, 2) / (2 * Math.pow(pointAdjustStd, 2));
        return pointAdjustScalar * Math.exp(exponent);
    }

    public Command getRunCommand() {
        return runCommand;
    }

    public Pose2d getTargetPose() {
        Logger.recordOutput(name + "/Pose", targetPose);
        return targetPose;
    }

    public double getExpectedPoints() {
        return expectedPoints;
    }

    public double getExpectedRP() {
        return expectedRP;
    }

    public String getName() {
        return name;
    }
}
