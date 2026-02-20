package frc.robot.Graph;

import java.util.*;
import java.util.function.Supplier;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
import frc.robot.Constants;

public class Helpers {

    private static Supplier<Pose2d> robotPoseSupplier;
    private static FieldGrid field;
    private static Supplier<List<Pose2d>> opposingRobotsSupplier;
    private static Map<String, List<Translation2d>> pathCache = new HashMap<>();

    private static double[][] gScore;
    private static boolean[][] closed;
    private static int[][] parentR;
    private static int[][] parentC;

    private static final double SQRT2 = 1.41421356237;
    private static final double OCTILE_FACTOR = SQRT2 - 2.0;

    private static class OpenNode implements Comparable<OpenNode> {
        int r, c;
        double f;

        OpenNode(int r, int c, double f) {
            this.r = r;
            this.c = c;
            this.f = f;
        }

        @Override
        public int compareTo(OpenNode o) {
            return Double.compare(this.f, o.f);
        }
    }

    public static void initialize(Supplier<Pose2d> robotPose, String fieldJson, Supplier<List<Pose2d>> opposingRobots) {

        robotPoseSupplier = robotPose;

        field = new FieldGrid(
                Filesystem.getDeployDirectory()
                        .toPath()
                        .resolve(fieldJson)
                        .toString());

        opposingRobotsSupplier = opposingRobots;

        int rows = field.getRows();
        int cols = field.getCols();

        gScore = new double[rows][cols];
        closed = new boolean[rows][cols];
        parentR = new int[rows][cols];
        parentC = new int[rows][cols];
    }

    public static double getPathTime(Pose2d targetPose, iVertex vertex) {

        long totalStart = System.nanoTime();

        Pose2d startPose = robotPoseSupplier.get();

        int[] startCell = field.toCell(startPose.getTranslation());
        int[] targetCell = field.toCell(targetPose.getTranslation());

        long aStarStart = System.nanoTime();
        List<Translation2d> path = aStarPath(startCell, targetCell, vertex);
        long aStarEnd = System.nanoTime();

        pathCache.put(vertex.getName(), path);

        if (path.isEmpty()) {

            Logger.recordOutput(vertex.getName() + "/A*/Success", false);
            Logger.recordOutput(vertex.getName() + "/A*/TotalTimeMs",
                    (System.nanoTime() - totalStart) / 1e6);

            return Double.POSITIVE_INFINITY;
        }

        long distStart = System.nanoTime();

        double distance = 0.0;
        Translation2d last = path.get(0);

        for (int i = 1; i < path.size(); i++) {
            Translation2d current = path.get(i);
            distance += last.getDistance(current);
            last = current;
        }

        long distEnd = System.nanoTime();

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

        long totalEnd = System.nanoTime();

        // --- Logging ---
        Logger.recordOutput(vertex.getName() + "/A*/Success", true);
        Logger.recordOutput(vertex.getName() + "/A*/AStarTimeMs",
                (aStarEnd - aStarStart) / 1e6);
        Logger.recordOutput(vertex.getName() + "/A*/DistanceCalcTimeMs",
                (distEnd - distStart) / 1e6);
        Logger.recordOutput(vertex.getName() + "/A*/TotalTimeMs",
                (totalEnd - totalStart) / 1e6);
        Logger.recordOutput(vertex.getName() + "/A*/DistanceMeters", distance);
        Logger.recordOutput(vertex.getName() + "/A*/EstimatedTravelTimeSec", time);
        Logger.recordOutput(vertex.getName() + "/A*/PathLengthNodes", path.size());

        if (Constants.DEBUG_MODE) {
            long visualizationStart = System.nanoTime();
            logPath(path, vertex);
            long visualizationEnd = System.nanoTime();
            Logger.recordOutput(vertex.getName() + "/A*/Visualize", (visualizationEnd - visualizationStart) / 1e6);
        }

        return time;
    }

    private static List<Translation2d> aStarPath(
            int[] startCell,
            int[] targetCell,
            iVertex vertex) {

        final int rows = field.getRows();
        final int cols = field.getCols();

        final int startR = startCell[0];
        final int startC = startCell[1];
        final int targetR = targetCell[0];
        final int targetC = targetCell[1];

        PriorityQueue<OpenNode> open = new PriorityQueue<>();

        int nodesExpanded = 0;
        int nodesPushed = 0;

        for (int r = 0; r < rows; r++) {
            Arrays.fill(gScore[r], Double.POSITIVE_INFINITY);
            Arrays.fill(closed[r], false);
        }

        gScore[startR][startC] = 0.0;
        parentR[startR][startC] = -1;
        parentC[startR][startC] = -1;

        open.add(new OpenNode(
                startR,
                startC,
                heuristic(startR, startC, targetR, targetC)));
        nodesPushed++;

        final int[][] neighbors = {
                { -1, 0 }, { 1, 0 }, { 0, -1 }, { 0, 1 },
                { -1, -1 }, { -1, 1 }, { 1, -1 }, { 1, 1 }
        };

        while (!open.isEmpty()) {

            OpenNode current = open.poll();

            if (closed[current.r][current.c])
                continue;

            nodesExpanded++;

            if (current.r == targetR && current.c == targetC)
                break;

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

                double moveCost = (d[0] != 0 && d[1] != 0) ? SQRT2 : 1.0;

                double gNew = gScore[current.r][current.c] + moveCost;

                if (gNew >= gScore[nr][nc])
                    continue;

                gScore[nr][nc] = gNew;
                parentR[nr][nc] = current.r;
                parentC[nr][nc] = current.c;

                double fNew = gNew + heuristic(nr, nc, targetR, targetC);

                open.add(new OpenNode(nr, nc, fNew));
                nodesPushed++;
            }
        }

        Logger.recordOutput(vertex.getName() + "/A*/NodesExpanded", nodesExpanded);
        Logger.recordOutput(vertex.getName() + "/A*/NodesPushed", nodesPushed);

        if (gScore[targetR][targetC] == Double.POSITIVE_INFINITY)
            return Collections.emptyList();

        List<Translation2d> path = new ArrayList<>();

        int r = targetR;
        int c = targetC;

        while (r != -1) {
            path.add(field.toField(r, c));
            int pr = parentR[r][c];
            int pc = parentC[r][c];
            r = pr;
            c = pc;
        }

        Collections.reverse(path);
        return path;
    }

    private static double heuristic(int r1, int c1, int r2, int c2) {
        int dx = Math.abs(r1 - r2);
        int dy = Math.abs(c1 - c2);
        return (dx + dy) + OCTILE_FACTOR * Math.min(dx, dy);
    }

    private static void logPath(List<Translation2d> path, iVertex vertex) {
        if (path.isEmpty())
            return;

        int nPoints = 50;
        nPoints = Math.min(nPoints, path.size());
        nPoints = Math.max(nPoints, 1);

        double[] flattened = new double[nPoints * 2];

        if (nPoints == 1) {
            Translation2d t = path.get(0);
            flattened[0] = t.getX();
            flattened[1] = t.getY();
        } else {
            for (int i = 0; i < nPoints; i++) {
                int idx = i * (path.size() - 1) / (nPoints - 1);
                Translation2d t = path.get(idx);
                flattened[i * 2] = t.getX();
                flattened[i * 2 + 1] = t.getY();
            }
        }

        Logger.recordOutput(vertex.getName() + "/AStarPathFlattened", flattened);
    }

    public static void updateBotPosistions() {
        List<Pose2d> robots = opposingRobotsSupplier.get();

        for (Pose2d robot : robots) {
            int[] center = field.toCell(robot.getTranslation());
            int centerR = center[0];
            int centerC = center[1];

            for (int dr = -Constants.robotBlockRadius; dr <= Constants.robotBlockRadius; dr++) {
                for (int dc = -Constants.robotBlockRadius; dc <= Constants.robotBlockRadius; dc++) {
                    field.setPassable(centerR + dr, centerC + dc, false);
                }
            }
        }
    }

    public static double getMatchTime() {
        double timeLeft = Math.max(0, DriverStation.getMatchTime());
        return DriverStation.isAutonomous()
                ? timeLeft + 135
                : timeLeft;
    }

    public static double getRobotScore(String vertexName) {
        List<Translation2d> path = pathCache.get(vertexName);

        if (path == null || path.isEmpty())
            return 0.0;

        List<Pose2d> robots = opposingRobotsSupplier.get();

        double totalScore = 0.0;

        for (Pose2d enemyRobot : robots) {
            double minDistance = Double.POSITIVE_INFINITY;

            for (Translation2d pathPoint : path) {
                double dist = pathPoint.getDistance(enemyRobot.getTranslation());
                minDistance = Math.min(minDistance, dist);
            }

            totalScore += Math.exp(-minDistance);
        }

        return totalScore;
    }

}
