package frc.robot.Graph;

import java.io.IOException;
import java.nio.file.Files;
import java.nio.file.Paths;

import org.json.JSONArray;
import org.json.JSONObject;

import edu.wpi.first.math.geometry.Translation2d;

public class FieldGrid {

    private boolean[][] grid;
    private double fieldWidth;
    private double fieldHeight;
    private int rows;
    private int cols;

    /** Load grid from JSON file */
    public FieldGrid(String jsonPath) {
        loadFromJson(jsonPath);
    }

    /** Load JSON where each row is a string of 0/1 characters */
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

            // JSON top row = top of field
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

    /** Convert field coordinates (meters) to grid cell indices */
    public int[] toCell(Translation2d point) {
        int col = (int) (point.getX() / fieldWidth * cols);
        int row = (int) (point.getY() / fieldHeight * rows);

        col = Math.max(0, Math.min(cols - 1, col));
        row = Math.max(0, Math.min(rows - 1, row));
        return new int[]{row, col};
    }

    /** Convert grid cell indices to field coordinates (center of cell) */
    public Translation2d toField(int row, int col) {
        double x = (col + 0.5) * fieldWidth / cols;
        double y = (row + 0.5) * fieldHeight / rows;
        return new Translation2d(x, y);
    }

    /** Check if a given grid cell is passable */
    public boolean isPassable(int row, int col) {
        if (row < 0 || row >= rows || col < 0 || col >= cols) return false;
        return grid[row][col];
    }

    /** Check if a Translation2d / pose is in a passable cell */
    public boolean isPassable(Translation2d point) {
        int[] cell = toCell(point);
        return isPassable(cell[0], cell[1]);
    }

    /** Mark a grid cell blocked/unblocked */
    public void setPassable(int row, int col, boolean passable) {
        if (row >= 0 && row < rows && col >= 0 && col < cols) {
            grid[row][col] = passable;
        }
    }

    /** Mark the cell at a Translation2d blocked/unblocked */
    public void setPassable(Translation2d point, boolean passable) {
        int[] cell = toCell(point);
        setPassable(cell[0], cell[1], passable);
    }

    /** Get the width of one grid cell in meters */
    public double getCellWidth() {
        return fieldWidth / cols;
    }

    /** Get the height of one grid cell in meters */
    public double getCellHeight() {
        return fieldHeight / rows;
    }

    // Getters
    public double getFieldWidth() { return fieldWidth; }
    public double getFieldHeight() { return fieldHeight; }
    public int getRows() { return rows; }
    public int getCols() { return cols; }
    public boolean[][] getGrid() { return grid; }

    /** Optional: Print the grid to console for debugging */
    public void printGrid() {
        for (int r = 0; r < rows; r++) {
            for (int c = 0; c < cols; c++) {
                System.out.print(grid[r][c] ? "1" : "0");
            }
            System.out.println();
        }
    }
}
