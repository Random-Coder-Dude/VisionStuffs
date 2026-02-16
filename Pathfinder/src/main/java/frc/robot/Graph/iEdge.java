package frc.robot.Graph;

public interface iEdge {
    iVertex getFromVertex();

    iVertex getToVertex();

    boolean getIsEnabled();

    void setIsEnabled(boolean enabled);

    double getWeight();
}
