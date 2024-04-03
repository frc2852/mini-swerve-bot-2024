package frc.robot.constants;

import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import frc.robot.util.vision.CameraConfiguration;

public final class VisionConstants {
      public static class CameraTracking {

    public static final CameraConfiguration APRIL_TAG_CAMERA_CONFIG = new CameraConfiguration("AprilTagCamera",
        new Transform3d(new Translation3d(0.5, 0.0, 0.5), new Rotation3d(0, 0, 0)));

    public static final CameraConfiguration GAME_PIECE_CAMERA_CONFIG = new CameraConfiguration(
        "GamePieceCamera",
        new Transform3d(new Translation3d(0.5, 0.0, 0.5), new Rotation3d(0, 0, 0)));

    public static final int TOTAL_TAGS = 16;
  }

  public static class LEDConstants {
    // I2C Addresses for the Arduino controllers
    public static final int ARDUINO_LEFT_ADDRESS = 0x01;
    public static final int ARDUINO_RIGHT_ADDRESS = 0x02;
  }
}
