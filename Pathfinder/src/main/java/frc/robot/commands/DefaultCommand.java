package frc.robot.commands;

import java.util.List;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.Graph.Helpers;
import frc.robot.Graph.adjMatrix;
import frc.robot.Graph.iVertex;
import frc.robot.Graph.standardVertex;
import frc.robot.subsystems.ExampleSubsystem;

public class DefaultCommand extends Command {

    private iVertex v1;
    private iVertex v2;
    private iVertex v3;
    private Pose2d robot1;
    private Pose2d robot2;
    private adjMatrix matrix;

    public DefaultCommand(ExampleSubsystem m_exampleSubsystem) {
        v1 = new standardVertex(0, 1, 0, null, new Pose2d(0, 6, new Rotation2d()), 50, 0,
                "Shoot");
        v2 = new standardVertex(0, 1, 0, null, new Pose2d(5, 3, new Rotation2d()), 10, 0,
                "Climb");
        v3 = new standardVertex(0, 1, 0, null, new Pose2d(8, 6, new Rotation2d()), 0, 0,
                "Defense");
        robot1 = new Pose2d(1.65, 3.7, Rotation2d.fromDegrees(90));
        robot2 = new Pose2d(4.6, 5.6, Rotation2d.fromDegrees(-45));
        matrix = new adjMatrix(v1, v2, v3);

        addRequirements(m_exampleSubsystem);
    }

    @Override
    public void initialize() {
        Logger.recordOutput("Robot 1", robot1);
        Logger.recordOutput("Robot 2", robot2);
        Helpers.initialize(() -> Constants.robotPose, "gameField.json", () -> List.of(robot1, robot2));
    }

    @Override
    public void execute() {
        Constants.time = DriverStation.getMatchTime();
        Logger.recordOutput("robotPose", Constants.robotPose);
        Helpers.clearCachedPaths();
        Helpers.updateBotPosistions();
        matrix.updateWeights();
    }

    @Override
    public boolean isFinished() {
        return false;
    }

    @Override
    public void end(boolean interrupted) {

    }
}
