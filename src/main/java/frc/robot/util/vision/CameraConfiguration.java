package frc.robot.util.vision;

import edu.wpi.first.math.geometry.Transform3d;

public class CameraConfiguration {
  private final String cameraName;
  private final Transform3d cameraPosition;

  public CameraConfiguration(String cameraName, Transform3d cameraPosition) {
    this.cameraName = cameraName;
    this.cameraPosition = cameraPosition;
  }

  public String getCameraName() {
    return cameraName;
  }

  public Transform3d getCameraPosition() {
    return cameraPosition;
  }
}
