package frc.robot.util.vision;

public class TargetInfo {
    private final double angle;
    private final double distance;

    public TargetInfo(double angle, double distance) {
        this.angle = angle;
        this.distance = distance;
    }

    public double getAngle() {
        return angle;
    }

    public double getDistance() {
        return distance;
    }
}
