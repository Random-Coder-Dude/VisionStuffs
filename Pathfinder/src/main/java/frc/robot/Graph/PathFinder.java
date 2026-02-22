package frc.robot.Graph;

import java.util.*;

public class PathFinder {

    public static List<iVertex> findBestPath(adjMatrix matrix, iVertex startVertex, int numSteps) {
        iVertex[] vertices = matrix.getVertices();
        int startIndex = getVertexIndex(vertices, startVertex);
        
        if (startIndex == -1) {
            return Collections.emptyList();
        }

        // DP: dp[step][vertex] = (bestScore, previousVertex)
        double[][] dp = new double[numSteps + 1][vertices.length];
        int[][] parent = new int[numSteps + 1][vertices.length];
        
        // Initialize: all states start at negative infinity except start vertex at step 0
        for (int step = 0; step <= numSteps; step++) {
            Arrays.fill(dp[step], Double.POSITIVE_INFINITY);
            Arrays.fill(parent[step], -1);
        }
        dp[0][startIndex] = 0.0;

        // Fill DP table
        for (int step = 0; step < numSteps; step++) {
            for (int from = 0; from < vertices.length; from++) {
                if (dp[step][from] == Double.POSITIVE_INFINITY) continue;

                for (int to = 0; to < vertices.length; to++) {
                    double edgeWeight = matrix.getWeight(vertices[from], vertices[to]);
                    
                    if (edgeWeight == Double.POSITIVE_INFINITY) continue;

                    double newScore = dp[step][from] + edgeWeight;
                    
                    if (newScore < dp[step + 1][to]) {
                        dp[step + 1][to] = newScore;
                        parent[step + 1][to] = from;
                    }
                }
            }
        }

        // Find best ending vertex at step numSteps
        double bestScore = Double.POSITIVE_INFINITY;
        int bestEndVertex = -1;
        
        for (int v = 0; v < vertices.length; v++) {
            if (dp[numSteps][v] < bestScore) {
                bestScore = dp[numSteps][v];
                bestEndVertex = v;
            }
        }

        if (bestEndVertex == -1) {
            return Collections.emptyList();
        }

        // Reconstruct path
        List<iVertex> path = new ArrayList<>();
        int current = bestEndVertex;
        
        for (int step = numSteps; step >= 0; step--) {
            path.add(vertices[current]);
            current = parent[step][current];
            if (current == -1) break;
        }

        Collections.reverse(path);
        return path;
    }

    private static int getVertexIndex(iVertex[] vertices, iVertex target) {
        for (int i = 0; i < vertices.length; i++) {
            if (vertices[i] == target) return i;
        }
        return -1;
    }
}