package frc.robot.Graph;

import java.io.IOException;
import java.nio.file.Files;
import java.nio.file.Paths;

import org.json.JSONArray;
import org.json.JSONObject;
import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.geometry.Translation2d;

public class FieldGrid {

    private boolean[][] grid;
    private boolean[][] originalGrid;
    private double fieldWidth;
    private double fieldHeight;
    private int rows;
    private int cols;

    public FieldGrid(String jsonPath) {
        long start = System.nanoTime();
        loadFromJson(jsonPath);
        originalGrid = new boolean[rows][cols];
        for (int r = 0; r < rows; r++)
            originalGrid[r] = grid[r].clone();
        long end = System.nanoTime();
        Logger.recordOutput("Json Time", ((end - start) / 1e6) + " ms");
    }

    private void loadFromJson(String jsonPath) {
        try {
            String content = new String(Files.readAllBytes(Paths.get(jsonPath)));
            JSONObject json = new JSONObject(content);

            fieldWidth = json.getDouble("fieldWidth");
            fieldHeight = json.getDouble("fieldHeight");
            rows = json.getInt("rows");
            cols = json.getInt("cols");

            JSONArray gridArray = json.getJSONArray("grid");
            grid = new boolean[rows][cols];

            for (int r = 0; r < rows; r++) {
                String rowStr = gridArray.getString(r);
                for (int c = 0; c < cols; c++) {
                    grid[r][c] = rowStr.charAt(c) == '1';
                }
            }

        } catch (IOException e) {
            System.err.println("Failed to load field grid: " + e.getMessage());
            grid = new boolean[0][0];
        }
    }

    public int[] toCell(Translation2d point) {
        int col = (int) (point.getX() / fieldWidth * cols);
        int row = (int) (point.getY() / fieldHeight * rows);

        col = Math.max(0, Math.min(cols - 1, col));
        row = Math.max(0, Math.min(rows - 1, row));
        return new int[] { row, col };
    }

    public Translation2d toField(int row, int col) {
        double x = (col + 0.5) * fieldWidth / cols;
        double y = (row + 0.5) * fieldHeight / rows;
        return new Translation2d(x, y);
    }

    public boolean isPassable(int row, int col) {
        if (row < 0 || row >= rows || col < 0 || col >= cols)
            return false;
        return grid[row][col];
    }

    public boolean isPassable(Translation2d point) {
        int[] cell = toCell(point);
        return isPassable(cell[0], cell[1]);
    }

    public void setPassable(int row, int col, boolean passable) {
        if (row >= 0 && row < rows && col >= 0 && col < cols) {
            grid[row][col] = passable;
        }
    }

    public void setPassable(Translation2d point, boolean passable) {
        int[] cell = toCell(point);
        setPassable(cell[0], cell[1], passable);
    }

    public double getCellWidth() {
        return fieldWidth / cols;
    }

    public double getCellHeight() {
        return fieldHeight / rows;
    }

    public double getFieldWidth() {
        return fieldWidth;
    }

    public double getFieldHeight() {
        return fieldHeight;
    }

    public int getRows() {
        return rows;
    }

    public int getCols() {
        return cols;
    }

    public boolean[][] getGrid() {
        return grid;
    }

    public void resetGrid() {
        for (int r = 0; r < rows; r++)
            grid[r] = originalGrid[r].clone();
    }

    public void printGrid() {
        for (int r = 0; r < rows; r++) {
            for (int c = 0; c < cols; c++) {
                System.out.print(grid[r][c] ? "1" : "0");
            }
            System.out.println();
        }
    }
}
