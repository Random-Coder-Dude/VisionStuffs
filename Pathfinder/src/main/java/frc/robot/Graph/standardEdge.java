package frc.robot.Graph;

import frc.robot.Constants;

public class standardEdge implements iEdge {
    private iVertex fromVertex;
    private iVertex toVertex;
    private boolean enabled;
    private double cachedTimeTerm;

    public standardEdge(iVertex fromVertex, iVertex toVertex) {
        this.fromVertex = fromVertex;
        this.toVertex = toVertex;
        enabled = true;
        cachedTimeTerm = 0.0;
    }

    public iVertex getFromVertex() {
        return fromVertex;
    }

    public iVertex getToVertex() {
        return toVertex;
    }

    public boolean getIsEnabled() {
        return enabled;
    }

    public void setIsEnabled(boolean enabled) {
        this.enabled = enabled;
    }

    public double getWeight() {
        if (!enabled) {
            return Double.POSITIVE_INFINITY;
        }

        double timeTerm = Constants.k1 * Helpers.getPathTime(toVertex.getTargetPose());
        if (timeTerm == Double.POSITIVE_INFINITY) {
            timeTerm = cachedTimeTerm;
        } else if (timeTerm == Double.NEGATIVE_INFINITY) {
            return Double.POSITIVE_INFINITY;
        } else {
            cachedTimeTerm = timeTerm;
        }
        double pointTerm = Constants.k2 * toVertex.getExpectedPoints();
        double miscTerm = (Helpers.getRobotScore() + toVertex.getExpectedRP()
                + toVertex.pointAdjust(Helpers.getMatchTime())) / Constants.k3;
        return cachedTimeTerm - pointTerm + miscTerm;
    }
}
