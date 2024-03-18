package frc.robot.subsystems.vision;

import org.photonvision.PhotonCamera;
import org.photonvision.common.hardware.VisionLEDMode;
import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.PowerHubSubsystem;
import frc.robot.util.vision.CameraConfiguration;

/**
 * Subsystem for detecting game pieces using PhotonVision.
 * This subsystem integrates with a PhotonCamera to detect and track game
 * pieces.
 */
public class GamePieceDetectionSubsystem extends SubsystemBase {

  private final PhotonCamera camera;
  private final CameraConfiguration cameraConfig;
  private final PowerHubSubsystem powerHubSubsystem;

  private boolean trackingMode = false;
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
  public GamePieceDetectionSubsystem(CameraConfiguration cameraConfig, PowerHubSubsystem ledSubsystem) {
    if (cameraConfig == null) {
      throw new IllegalArgumentException("Camera configuration cannot be null.");
    }

    this.powerHubSubsystem = ledSubsystem;
    this.cameraConfig = cameraConfig;
    this.camera = initializePhotonVisionCamera();
  }

  /**
   * Initializes the PhotonVision camera.
   * <p>
   * This method creates a new PhotonCamera instance using the camera name from the camera configuration.
   * It checks if the camera is connected and reports an error to the DriverStation if the connection fails.
   * If the camera is successfully connected, the driver mode is set to false, and the LED mode is turned off.
   * 
   * @return The initialized PhotonCamera if successful, or null if the camera is not connected.
   */
  private PhotonCamera initializePhotonVisionCamera() {
    PhotonCamera newCamera = new PhotonCamera(cameraConfig.getCameraName());

    // Check if the newCamera object is not null and is connected, otherwise report error
    if (newCamera == null || !newCamera.isConnected()) {
      DriverStation.reportError("April Tag Camera connection failed, camera initialization aborted.", false);
      return null;
    }

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
    // Check if the camera is not connected and report a warning
    if (camera != null && !camera.isConnected()) {
      DriverStation.reportWarning("Camera is not connected. Cannot update game piece visibility.", false);
      currentTarget = null;
      SmartDashboard.putBoolean("Game Piece Visible", false);
      return;
    }

    // If not in tracking mode, simply return
    if (!trackingMode) {
      return;
    }

    // When in tracking mode and camera is connected, update target visibility
    var result = camera.getLatestResult();
    if (!result.hasTargets()) {
      SmartDashboard.putBoolean("Game Piece Visible", false);
      currentTarget = null;
    } else {
      currentTarget = result.getBestTarget();
      SmartDashboard.putBoolean("Game Piece Visible", true);
    }
  }

  /**
   * Enables the tracking mode for detecting game pieces.
   * When tracking mode is enabled, the system actively seeks and tracks game pieces,
   * and the LED subsystem is set to high beam mode for enhanced visibility.
   */
  public void enableTrackingMode() {
    trackingMode = true;
    powerHubSubsystem.highBeamsOn();
  }

  /**
   * Disables the tracking mode for detecting game pieces.
   * When tracking mode is disabled, the system stops tracking game pieces,
   * turns off the high beam mode in the LED subsystem, and updates the SmartDashboard
   * to indicate that the game piece is no longer visible.
   */
  public void disableTrackingMode() {
    currentTarget = null;
    trackingMode = false;
    powerHubSubsystem.highBeamsOff();
    SmartDashboard.putBoolean("Game Piece Visible", false);
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
   * Retrieves the yaw angle of the currently visible game piece.
   * If no game piece is visible, returns 0.0.
   *
   * @return The yaw angle of the visible game piece, or 0.0 if no game piece is visible.
   */
  public double getTargetYaw() {
    return currentTarget != null ? currentTarget.getYaw() : 0.0;
  }

  /**
   * Retrieves the area of the currently visible game piece.
   * If no game piece is visible, returns 0.0.
   *
   * @return The area of the visible game piece, or 0.0 if no game piece is visible.
   */
  public double getTargetArea() {
    return currentTarget != null ? currentTarget.getArea() : 0.0;
  }
}
