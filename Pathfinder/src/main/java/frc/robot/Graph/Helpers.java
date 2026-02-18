package frc.robot.Graph;

import java.util.*;
import java.util.function.Supplier;
import java.util.stream.Collectors;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
import frc.robot.Constants;

public class Helpers {

    private static Supplier<Pose2d> robotPoseSupplier;
    private static FieldGrid field;

    public static void initialize(Supplier<Pose2d> robotPose, String fieldJson) {
        robotPoseSupplier = robotPose;
        field = new FieldGrid(Filesystem.getDeployDirectory().toPath().resolve(fieldJson).toString());
    }

    public static double getPathTime(Pose2d targetPose, iVertex vertex) {
        Pose2d startPose = robotPoseSupplier.get();
        int[] startCell = field.toCell(startPose.getTranslation());
        int[] targetCell = field.toCell(targetPose.getTranslation());

        List<Translation2d> path = aStarPath(startCell, targetCell);
        if (path.isEmpty())
            return Double.POSITIVE_INFINITY;

        if (Constants.DEBUG_MODE) {
            logPath(path, vertex);
        }

        double distance = 0.0;
        Translation2d last = path.get(0);
        for (int i = 1; i < path.size(); i++) {
            distance += last.getDistance(path.get(i));
            last = path.get(i);
        }

        double vmax = Constants.PATH_CONSTRAINTS.maxVelocityMPS();
        double amax = Constants.PATH_CONSTRAINTS.maxAccelerationMPSSq();
        double tAccel = vmax / amax;
        double dAccel = 0.5 * amax * tAccel * tAccel;

        double time;
        if (2 * dAccel >= distance) {
            time = 2 * Math.sqrt(distance / amax);
        } else {
            double dCruise = distance - 2 * dAccel;
            double tCruise = dCruise / vmax;
            time = 2 * tAccel + tCruise;
        }

        return time;
    }

    private static List<Translation2d> aStarPath(int[] startCell, int[] targetCell) {
        final int rows = field.getRows();
        final int cols = field.getCols();

        final int startR = startCell[0];
        final int startC = startCell[1];
        final int targetR = targetCell[0];
        final int targetC = targetCell[1];

        class Node implements Comparable<Node> {
            int r, c;
            double g, f;
            Node parent;

            Node(int r, int c, double g, double f, Node parent) {
                this.r = r;
                this.c = c;
                this.g = g;
                this.f = f;
                this.parent = parent;
            }

            @Override
            public int compareTo(Node o) {
                return Double.compare(this.f, o.f);
            }
        }

        PriorityQueue<Node> open = new PriorityQueue<>();
        boolean[][] closed = new boolean[rows][cols];

        double[][] gScore = new double[rows][cols];
        for (int r = 0; r < rows; r++) {
            Arrays.fill(gScore[r], Double.POSITIVE_INFINITY);
        }

        gScore[startR][startC] = 0.0;

        open.add(new Node(
                startR,
                startC,
                0.0,
                heuristic(startR, startC, targetR, targetC),
                null));

        final int[][] neighbors = {
                { -1, 0 }, { 1, 0 }, { 0, -1 }, { 0, 1 },
                { -1, -1 }, { -1, 1 }, { 1, -1 }, { 1, 1 }
        };

        Node endNode = null;

        while (!open.isEmpty()) {
            Node current = open.poll();

            if (closed[current.r][current.c])
                continue;

            if (current.r == targetR && current.c == targetC) {
                endNode = current;
                break;
            }

            closed[current.r][current.c] = true;

            for (int[] d : neighbors) {
                int nr = current.r + d[0];
                int nc = current.c + d[1];

                if (nr < 0 || nr >= rows || nc < 0 || nc >= cols)
                    continue;

                if (!field.isPassable(nr, nc))
                    continue;

                if (closed[nr][nc])
                    continue;

                double moveCost = (d[0] != 0 && d[1] != 0) ? Math.sqrt(2) : 1.0;
                double gNew = current.g + moveCost;

                if (gNew >= gScore[nr][nc])
                    continue;

                gScore[nr][nc] = gNew;

                double fNew = gNew + heuristic(nr, nc, targetR, targetC);

                open.add(new Node(nr, nc, gNew, fNew, current));
            }
        }

        if (endNode == null)
            return Collections.emptyList();

        List<Translation2d> path = new ArrayList<>();
        Node n = endNode;
        while (n != null) {
            path.add(field.toField(n.r, n.c));
            n = n.parent;
        }

        Collections.reverse(path);
        return path;
    }

    private static double heuristic(int r1, int c1, int r2, int c2) {
        double dr = r1 - r2;
        double dc = c1 - c2;
        return Math.sqrt(dr * dr + dc * dc);
    }

    private static void logPath(List<Translation2d> path, iVertex vertex) {
        List<Pose2d> waypoints = path.stream()
                .map(t -> new Pose2d(t, new Rotation2d()))
                .collect(Collectors.toList());

        TrajectoryConfig config = new TrajectoryConfig(
                Constants.PATH_CONSTRAINTS.maxVelocityMPS(),
                Constants.PATH_CONSTRAINTS.maxAccelerationMPSSq());

        Trajectory trajectory = TrajectoryGenerator.generateTrajectory(waypoints, config);

        Logger.recordOutput(vertex.getName() + "/Planned/Trajectory", trajectory);
    }

    public static double getRobotScore() {
        return 0.0;
    }

    public static double getMatchTime() {
        double timeLeft = Math.max(0, DriverStation.getMatchTime());
        return DriverStation.isAutonomous() ? timeLeft + 135 : timeLeft;
    }
}
