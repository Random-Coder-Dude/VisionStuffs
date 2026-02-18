package frc.robot.Graph;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.Command;

public interface iVertex {
    double pointAdjust(double time);

    Command getRunCommand();

    Pose2d getTargetPose();

    double getExpectedPoints();

    double getExpectedRP();

    String getName();
}
