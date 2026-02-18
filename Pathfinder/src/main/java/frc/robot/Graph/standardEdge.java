package frc.robot.Graph;

import org.littletonrobotics.junction.Logger;

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

        double timeTerm = Constants.k1 * Helpers.getPathTime(toVertex.getTargetPose(), toVertex);

        if (Double.isInfinite(timeTerm)) {
            Logger.recordOutput(toVertex.getName() + "/Score", Double.POSITIVE_INFINITY);
            return Double.POSITIVE_INFINITY;
        }

        cachedTimeTerm = timeTerm;

        double pointTerm = Constants.k2 * toVertex.getExpectedPoints();
        double miscTerm = (Helpers.getRobotScore() - toVertex.getExpectedRP()
                - toVertex.pointAdjust(Helpers.getMatchTime())) / Constants.k3;

        Logger.recordOutput(toVertex.getName() + "/Time Term", cachedTimeTerm);
        Logger.recordOutput(toVertex.getName() + "/Point Term", pointTerm);
        Logger.recordOutput(toVertex.getName() + "/Misc Term", miscTerm);
        Logger.recordOutput(toVertex.getName() + "/Score", cachedTimeTerm - pointTerm + miscTerm);
        return cachedTimeTerm - pointTerm + miscTerm;
    }
}
