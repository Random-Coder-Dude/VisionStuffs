package frc.robot.Graph;

import java.util.*;
import java.util.function.Supplier;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
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

    /**
     * Calculates the fastest path time to a target pose considering obstacles
     * Uses A* on the field grid and trapezoidal motion profiling
     */
    public static double getPathTime(Pose2d targetPose) {
        Pose2d startPose = robotPoseSupplier.get();
        int[] startCell = field.toCell(startPose.getTranslation());
        int[] targetCell = field.toCell(targetPose.getTranslation());

        List<Translation2d> path = aStarPath(startCell, targetCell);
        if (path.isEmpty())
            return Double.POSITIVE_INFINITY;

        // Compute total path distance
        double distance = 0.0;
        Translation2d last = path.get(0);
        for (int i = 1; i < path.size(); i++) {
            distance += last.getDistance(path.get(i));
            last = path.get(i);
        }

        // Trapezoidal motion profile
        double vmax = Constants.PATH_CONSTRAINTS.maxVelocityMPS();
        double amax = Constants.PATH_CONSTRAINTS.maxAccelerationMPSSq();
        double tAccel = vmax / amax;
        double dAccel = 0.5 * amax * tAccel * tAccel;

        if (2 * dAccel >= distance) {
            // Triangular profile (never reaches max velocity)
            System.out.println(2 * Math.sqrt(distance / amax));
            return 2 * Math.sqrt(distance / amax);
        } else {
            // Trapezoidal profile
            double dCruise = distance - 2 * dAccel;
            double tCruise = dCruise / vmax;
            System.out.println(2 * tAccel + tCruise);
            return 2 * tAccel + tCruise;
        }
    }

    // Fast A* pathfinding
    private static List<Translation2d> aStarPath(int[] startCell, int[] targetCell) {
        int rows = field.getRows();
        int cols = field.getCols();

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

            public int compareTo(Node o) {
                return Double.compare(this.f, o.f);
            }

            public boolean equals(Object o) {
                return o instanceof Node n && n.r == r && n.c == c;
            }

            public int hashCode() {
                return Objects.hash(r, c);
            }
        }

        PriorityQueue<Node> open = new PriorityQueue<>();
        Map<String, Double> gScore = new HashMap<>();
        boolean[][] closed = new boolean[rows][cols];

        Node startNode = new Node(startCell[0], startCell[1], 0, heuristic(startCell, targetCell), null);
        open.add(startNode);
        gScore.put(startCell[0] + "," + startCell[1], 0.0);

        int[][] neighbors = { { -1, 0 }, { 1, 0 }, { 0, -1 }, { 0, 1 },
                { -1, -1 }, { -1, 1 }, { 1, -1 }, { 1, 1 } };

        Node endNode = null;
        while (!open.isEmpty()) {
            Node current = open.poll();
            if (current.r == targetCell[0] && current.c == targetCell[1]) {
                endNode = current;
                break;
            }
            if (closed[current.r][current.c])
                continue;
            closed[current.r][current.c] = true;

            for (int[] n : neighbors) {
                int nr = current.r + n[0];
                int nc = current.c + n[1];
                if (nr < 0 || nr >= rows || nc < 0 || nc >= cols)
                    continue;
                if (!field.isPassable(nr, nc))
                    continue;

                double moveCost = (n[0] != 0 && n[1] != 0) ? Math.sqrt(2) : 1.0;
                double gNew = current.g + moveCost;
                String key = nr + "," + nc;

                if (gScore.containsKey(key) && gNew >= gScore.get(key))
                    continue;

                gScore.put(key, gNew);
                open.add(new Node(nr, nc, gNew, gNew + heuristic(new int[] { nr, nc }, targetCell), current));
            }
        }

        if (endNode == null)
            return Collections.emptyList();

        // Reconstruct path
        List<Translation2d> path = new ArrayList<>();
        Node n = endNode;
        while (n != null) {
            path.add(field.toField(n.r, n.c));
            n = n.parent;
        }
        Collections.reverse(path);
        return path;
    }

    // Euclidean heuristic for A*
    private static double heuristic(int[] a, int[] b) {
        Translation2d posA = field.toField(a[0], a[1]);
        Translation2d posB = field.toField(b[0], b[1]);
        return posA.getDistance(posB);
    }

    public static double getRobotScore() {
        return 0.0;
    }

    public static double getMatchTime() {
        double timeLeft = Math.max(0, DriverStation.getMatchTime());
        return DriverStation.isAutonomous() ? timeLeft + 135 : timeLeft;
    }
}
