package frc.robot.subsystems.vision;

import org.photonvision.PhotonCamera;
import org.photonvision.common.hardware.VisionLEDMode;
import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.util.vision.CameraConfiguration;
import frc.robot.util.vision.TargetInfo;

/**
 * Subsystem for detecting game pieces using PhotonVision.
 * This subsystem integrates with a PhotonCamera to detect and track game
 * pieces.
 */
public class GamePieceDetection extends SubsystemBase {

  private final PhotonCamera camera;
  private final CameraConfiguration cameraConfig;

  private PhotonTrackedTarget currentTarget = null;

  /**
   * Constructs a new instance of the GamePieceDetectionSubsystem.
   * Initializes the subsystem with the specified camera configuration and LED subsystem.
   * This constructor sets up the PhotonVision camera for game piece detection and integrates
   * an LED subsystem for visual feedback or signaling.
   *
   * @param cameraConfig The configuration settings for the camera, essential for initializing the camera.
   * @param ledSubsystem An instance of LEDSubsystem for LED control and feedback.
   * @throws IllegalArgumentException if the cameraConfig parameter is null, ensuring that valid configuration is provided.
   */
  public GamePieceDetection(CameraConfiguration cameraConfig) {
    if (cameraConfig == null) {
      throw new IllegalArgumentException("Camera configuration cannot be null.");
    }

    this.cameraConfig = cameraConfig;
    this.camera = initializePhotonVisionCamera();
  }

  /**
   * Initializes the PhotonVision camera.
   * <p>
   * This method creates a new PhotonCamera instance using the camera name from the camera configuration.
   * The driver mode is set to false, and the LED mode is turned off.
   * 
   * @return The initialized PhotonCamera if successful, or null if the camera is not connected.
   */
  private PhotonCamera initializePhotonVisionCamera() {
    PhotonCamera newCamera = new PhotonCamera(cameraConfig.getCameraName());
    newCamera.setDriverMode(false);
    newCamera.setLED(VisionLEDMode.kOff);
    return newCamera;
  }

  /**
   * Periodically updates the game piece visibility status based on the camera's connectivity and tracking mode.
   * If the camera is not connected, a warning is reported, and the game piece visibility is set to false.
   * 
   * When the camera is connected, and the subsystem is in tracking mode, it updates the visibility status of the game piece.
   * This is determined based on whether the camera detects any targets. If targets are detected, the best target is identified
   * and the visibility is set to true; otherwise, it's set to false.
   */
  @Override
  public void periodic() {
    if (isCameraDisconnected()) {
      handleDisconnectedCamera();
      return;
    }

    updateTargetVisibility();
    SmartDashboard.putBoolean("Game Piece Visible", isGamePieceVisible());
  }

  /**
   * Checks if a game piece is currently visible to the camera.
   *
   * @return True if a game piece is visible, false otherwise.
   */
  public boolean isGamePieceVisible() {
    return currentTarget != null;
  }

  /**
   * Gets the angle and distance to the current target.
   *
   * @return A TargetInfo object containing the angle and distance to the target, or null if no target is visible.
   */
  public TargetInfo getTargetInfo() {
    if (currentTarget == null) {
      return null;
    }

    double targetAngle = currentTarget.getYaw();
    double targetDistance = calculateDistance(currentTarget); // You need to implement this based on your camera setup

    return new TargetInfo(targetAngle, targetDistance);
  }

  /**
   * Calculates the distance to the target based on camera data.
   * Implement this method based on your camera setup and target dimensions.
   *
   * @param target The detected target.
   * @return The distance to the target.
   */
  private double calculateDistance(PhotonTrackedTarget target) {
    // Example implementation, replace with actual calculation
    double targetHeight = 0.0508; // Height of the target in meters
    double cameraHeight = 1.2; // Height of the camera from the ground in meters
    double cameraAngle = 30.0; // Angle of the camera in degrees

    double angleToTarget = Math.toRadians(cameraAngle + target.getPitch());
    return (targetHeight - cameraHeight) / Math.tan(angleToTarget);
  }

  /**
   * Updates the visibility status of the game piece based on the latest camera result.
   * If the camera has detected targets, the best target is selected as the current target.
   * If no targets are detected, the current target is set to null.
   */
  private void updateTargetVisibility() {
    var result = camera.getLatestResult();
    if (!result.hasTargets()) {
      currentTarget = null;
    } else {
      currentTarget = result.getBestTarget();
    }
  }

  /**
   * Checks if the camera is disconnected.
   *
   * @return True if the camera is not connected, false otherwise.
   */
  private boolean isCameraDisconnected() {
    return camera != null && !camera.isConnected();
  }

  /**
   * Handles the situation when the camera is disconnected.
   * Sets the current target to null and updates the SmartDashboard to indicate that the game piece is not visible.
   */
  private void handleDisconnectedCamera() {
    currentTarget = null;
    SmartDashboard.putBoolean("Game Piece Visible", isGamePieceVisible());
  }
}
