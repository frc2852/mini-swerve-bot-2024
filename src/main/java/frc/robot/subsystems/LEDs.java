package frc.robot.subsystems;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.I2C;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.util.vision.Color;

public class LEDs extends SubsystemBase {

  // Sensors
  private I2C arduino;

  public LEDs() {
    try {
      // Initialize the I2C port for communication with the Arduino
      arduino = new I2C(I2C.Port.kOnboard, 0x10); // Adjust the I2C address as needed
    } catch (Exception e) {
      DriverStation.reportError("Error initializing LED I2C: " + e.getMessage(), true);
    }
  }

  /**
   * Sends a predefined color to the Arduino controlling the LEDs.
   * 
   * @param color The color to set the LEDs to.
   */
  public void setLEDColor(Color color) {
    if (arduino != null) {
      String colorString = String.format("%d,%d,%d", color.getRed(), color.getGreen(), color.getBlue());
      byte[] colorBytes = colorString.getBytes();
      arduino.writeBulk(colorBytes);
    }
  }
}
