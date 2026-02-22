package frc.robot.subsystems;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.config.ModuleConfig;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Twist2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class DriveSubsystem extends SubsystemBase {

    public DriveSubsystem() {
        RobotConfig config = new RobotConfig(
            60.0,
            6.0,
            new ModuleConfig(
                0.048,
                4.5,
                1.2,
                DCMotor.getKrakenX60(1).withReduction(6.75),
                60.0,
                1
            ),
            new Translation2d(0.3, 0.3),
            new Translation2d(0.3, -0.3),
            new Translation2d(-0.3, 0.3),
            new Translation2d(-0.3, -0.3)
        );

        AutoBuilder.configure(
            this::getPose,
            this::resetPose,
            this::getRobotRelativeSpeeds,
            (speeds, feedforwards) -> driveRobotRelative(speeds),
            new PPHolonomicDriveController(
                new PIDConstants(5.0, 0.0, 0.0),
                new PIDConstants(5.0, 0.0, 0.0)
            ),
            config,
            () -> {
                var alliance = DriverStation.getAlliance();
                return alliance.isPresent() && alliance.get() == DriverStation.Alliance.Red;
            },
            this
        );
    }

    public Pose2d getPose() { return Constants.robotPose; }

    public void resetPose(Pose2d newPose) { Constants.robotPose = newPose; }

    public ChassisSpeeds getRobotRelativeSpeeds() { return new ChassisSpeeds(); }

    public void driveRobotRelative(ChassisSpeeds speeds) {
        Constants.robotPose = Constants.robotPose.exp(new Twist2d(
            speeds.vxMetersPerSecond * 0.02,
            speeds.vyMetersPerSecond * 0.02,
            speeds.omegaRadiansPerSecond * 0.02
        ));
    }
}