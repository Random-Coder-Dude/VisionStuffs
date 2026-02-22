package frc.robot.Graph;

import java.util.HashMap;
import java.util.Map;

public class adjMatrix {

    private iVertex[] vertices;
    private iEdge[][] edges;
    private double[][] weights;
    private Map<iVertex, Integer> vertexIndexMap;

    public adjMatrix(iVertex... vertices) {
        this.vertices = vertices;

        int n = vertices.length;
        edges = new iEdge[n][n];
        weights = new double[n][n];

        vertexIndexMap = new HashMap<>();
        for (int i = 0; i < n; i++) {
            vertexIndexMap.put(vertices[i], i);
        }

        for (int i = 0; i < n; i++) {
            for (int j = 0; j < n; j++) {
                if (i == j) {
                    edges[i][j] = null;
                    weights[i][j] = Double.POSITIVE_INFINITY;
                } else {
                    edges[i][j] = new standardEdge(vertices[i], vertices[j]);
                    weights[i][j] = Double.POSITIVE_INFINITY;
                }
            }
        }
    }

    public iEdge getEdge(int fromIndex, int toIndex) {
        return edges[fromIndex][toIndex];
    }

    public iEdge getEdge(iVertex from, iVertex to) {
        Integer fromIndex = vertexIndexMap.get(from);
        Integer toIndex = vertexIndexMap.get(to);
        if (fromIndex == null || toIndex == null)
            return null;
        return edges[fromIndex][toIndex];
    }

    public iVertex[] getVertices() {
        return vertices;
    }

    public iEdge[][] getEdges() {
        return edges;
    }

    public void setEdgeEnabled(iVertex from, iVertex to, boolean enabled) {
        iEdge edge = getEdge(from, to);
        if (edge != null) {
            edge.setIsEnabled(enabled);
        }
    }

    public double getWeight(iVertex from, iVertex to) {
        Integer fromIndex = vertexIndexMap.get(from);
        Integer toIndex = vertexIndexMap.get(to);

        if (fromIndex == null || toIndex == null) {
            return Double.POSITIVE_INFINITY;
        }

        return weights[fromIndex][toIndex];
    }

    public void updateWeights() {
        int n = vertices.length;

        for (int j = 0; j < n; j++) {
            Map<Class<?>, Double> cachedWeights = new HashMap<>();

            for (int i = 0; i < n; i++) {
                if (i == j) {
                    weights[i][j] = Double.POSITIVE_INFINITY;
                    continue;
                }

                iEdge edge = edges[i][j];
                if (edge == null) {
                    weights[i][j] = Double.POSITIVE_INFINITY;
                    continue;
                }

                if (edge.getIsEnabled()) {
                    Class<?> edgeType = edge.getClass();
                    weights[i][j] = cachedWeights.computeIfAbsent(edgeType, k -> edge.getWeight());
                } else {
                    weights[i][j] = Double.POSITIVE_INFINITY;
                }
            }
        }
    }

    public void printWeights() {
        int n = vertices.length;

        System.out.print("      ");
        for (int j = 0; j < n; j++) {
            System.out.printf("V%-8d", j + 1);
        }
        System.out.println();

        for (int i = 0; i < n; i++) {
            System.out.printf("V%-4d ", i + 1);
            for (int j = 0; j < n; j++) {
                double w = weights[i][j];
                if (w == Double.POSITIVE_INFINITY) {
                    System.out.printf("%-9s", "INF");
                } else {
                    System.out.printf("%-9.2f", w);
                }
            }
            System.out.println();
        }
    }
}