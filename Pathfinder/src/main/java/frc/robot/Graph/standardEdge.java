package frc.robot.Graph;

import edu.wpi.first.wpilibj.DriverStation;
import frc.robot.Constants;

public class standardEdge implements iEdge{
    private iVertex fromVertex;
    private iVertex toVertex;
    private boolean enabled;

    public standardEdge(iVertex fromVertex, iVertex toVertex) {
        this.fromVertex = fromVertex;
        this.toVertex = toVertex;
        enabled = true;
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

        double timeTerm = Constants.k1*Helpers.getPathTime(toVertex.getTargetPose());
        double pointTerm = Constants.k2*toVertex.getExpectedPoints();
        double miscTerm = (Helpers.getRobotScore() + toVertex.getExpectedRP() + toVertex.pointAdjust(Helpers.getMatchTime()))/Constants.k3;
        return timeTerm - pointTerm + miscTerm;
    }
}
