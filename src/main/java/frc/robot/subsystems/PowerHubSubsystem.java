// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.hal.PowerDistributionVersion;
import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.PowerDistribution.ModuleType;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.CanbusId;
import frc.robot.util.DataTracker;

/**
 * Represents the subsystem for managing power distribution within the robot.
 * This subsystem includes methods for monitoring and controlling various aspects
 * of power distribution such as battery voltage, total current, and high beams control.
 */
public class PowerHubSubsystem extends SubsystemBase {

  private final PowerDistribution powerDistribution;
  private final int totalchannels;

  private boolean initialized = false;
  private boolean deviceConnectionStatus = false;

  /**
   * Constructs a PowerHubSubsystem instance.
   * Initializes the power distribution module with specified CAN bus ID and module type,
   * and sets up the total number of channels available on the power distribution module.
   */
  public PowerHubSubsystem() {
    powerDistribution = new PowerDistribution(CanbusId.POWER_DISTRIBUTION_HUB, ModuleType.kRev);
    totalchannels = powerDistribution.getNumChannels();
  }

  /**
   * Called periodically during robot operation.
   * This method monitors and records various power metrics like battery voltage,
   * total current, and current for each channel if the device is connected.
   */
  @Override
  public void periodic() {
    try {
      if (isDeviceConnected()) {
        DataTracker.putNumber(getName(), "BatteryVoltage", powerDistribution.getVoltage(), true);
        DataTracker.putNumber(getName(), "Totalcurrent", powerDistribution.getTotalCurrent(), false);

        for (int i = 0; i < totalchannels; i++) {
          DataTracker.putNumber(getName(), "Ch" + i + "Current", powerDistribution.getCurrent(i), false);
        }
      }
    } catch (Exception e) {
      // ignore
    }
  }

  /**
   * Resets the total energy counter on the power distribution module.
   */
  public void reset() {
    powerDistribution.resetTotalEnergy();
  }

  /**
   * Turns off the high beams (switchable channels) on the power distribution module.
   */
  public void highBeamsOff() {
    powerDistribution.setSwitchableChannel(false);
  }

  /**
   * Turns on the high beams (switchable channels) on the power distribution module.
   */
  public void highBeamsOn() {
    powerDistribution.setSwitchableChannel(true);
  }

  /**
   * Checks if the CANSparkMax device is currently connected to the CAN bus.
   * 
   * This method attempts to retrieve the firmware version of the connected CANSparkMax device.
   * If the firmware version can be successfully retrieved and is not empty, it is assumed that
   * the device is connected. Otherwise, it is assumed that the device is not connected to the CAN bus.
   *
   * @return true if the device is connected (firmware version is retrievable and not empty),
   *         false otherwise.
   */
  private boolean isDeviceConnected() {
    if (!initialized) {
      // Attempt to get the firmware version
      PowerDistributionVersion firmwareVersion = powerDistribution.getVersion();

      // Check if firmwareVersion is not null and not empty
      deviceConnectionStatus = firmwareVersion != null && (firmwareVersion.firmwareMajor != 0 && firmwareVersion.firmwareMinor != 0 && firmwareVersion.hardwareMajor != 0 && firmwareVersion.hardwareMinor != 0);
      initialized = true;
    }
    return deviceConnectionStatus;
  }
}