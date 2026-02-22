package frc.robot.commands;

import java.util.List;

import org.littletonrobotics.junction.Logger;

import com.pathplanner.lib.auto.AutoBuilder;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants;
import frc.robot.Graph.Helpers;
import frc.robot.Graph.PathFinder;
import frc.robot.Graph.adjMatrix;
import frc.robot.Graph.iVertex;
import frc.robot.Graph.standardVertex;
import frc.robot.subsystems.ExampleSubsystem;

public class DefaultCommand extends Command {

    private iVertex v1;
    private iVertex v2;
    private iVertex v3;
    private iVertex currentVertex;
    private iVertex currentlyScheduledVertex;
    private Pose2d robot2;
    private adjMatrix matrix;
    private Command activeCommand;

    public DefaultCommand(ExampleSubsystem m_exampleSubsystem) {
        v1 = new standardVertex(0, 1, 0, new SequentialCommandGroup(
                new InstantCommand(() -> System.out.println("Shoot Vertex Ran")),
                new WaitCommand(1.5)),
                new Pose2d(0, 6, new Rotation2d()), 50, 0,
                "Shoot");
        v2 = new standardVertex(0, 1, 0, new InstantCommand(() -> System.out.println("Climb Vertex Ran")),
                new Pose2d(5, 3, new Rotation2d()), 10, 0,
                "Climb");
        v3 = new standardVertex(0, 1, 0, new InstantCommand(() -> System.out.println("Defense Vertex Ran")),
                new Pose2d(8, 6, new Rotation2d()), 0, 0,
                "Defense");
        robot2 = new Pose2d(4.6, 5.6, Rotation2d.fromDegrees(-45));
        matrix = new adjMatrix(v1, v2, v3);
        currentVertex = v1;
        currentlyScheduledVertex = null;
        activeCommand = null;

        addRequirements(m_exampleSubsystem);
    }

    @Override
    public void initialize() {
        Logger.recordOutput("Robot 2", robot2);
        Helpers.initialize(() -> Constants.robotPose, "gameField.json", () -> List.of(Constants.robot1, robot2));
    }

    @Override
    public void execute() {
        Logger.recordOutput("Robot 1", Constants.robot1);
        Helpers.updateBotPosistions();
        Helpers.clearCachedPaths();
        matrix.updateWeights();

        if (activeCommand != null && activeCommand.isScheduled()) {
            System.out.println("Skipping");
            return;
        }
        Constants.time = DriverStation.getMatchTime();
        Logger.recordOutput("robotPose", Constants.robotPose);

        List<iVertex> fullPath = PathFinder.findBestPath(matrix, currentVertex, Constants.numSteps);
        if (fullPath.size() < 2) {
            return;
        }
        iVertex next = fullPath.get(1);
        Logger.recordOutput("nextVertex", next.getName());
        if (next.getTargetPose().getTranslation()
                .getDistance(Constants.robotPose.getTranslation()) < Constants.arrivalThreshold) {
            currentVertex = next;
            activeCommand = currentVertex.getRunCommand();
            activeCommand.schedule();
            System.out.println("Running Command");
            return;
        }

        if (currentlyScheduledVertex != next) {
            currentlyScheduledVertex = next;
            AutoBuilder.pathfindToPose(next.getTargetPose(), Constants.PATH_CONSTRAINTS).schedule();
            System.out.println("Pathfinding");
            return;
        }
    }

    @Override
    public boolean isFinished() {
        return false;
    }

    @Override
    public void end(boolean interrupted) {

    }
}
