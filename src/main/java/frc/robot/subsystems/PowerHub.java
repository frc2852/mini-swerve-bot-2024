// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.PowerDistribution.ModuleType;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.Constants.CanbusId;

/**
 * Represents the subsystem for managing power distribution within the robot.
 * This subsystem includes methods for monitoring and controlling various aspects
 * of power distribution such as battery voltage, total current, and high beams control.
 */
public class PowerHub extends SubsystemBase {

  private final PowerDistribution powerDistribution;

  /**
   * Constructs a PowerHubSubsystem instance.
   * Initializes the power distribution module with specified CAN bus ID and module type,
   * and sets up the total number of channels available on the power distribution module.
   */
  public PowerHub() {
    powerDistribution = new PowerDistribution(CanbusId.POWER_DISTRIBUTION_HUB, ModuleType.kRev);
    highBeamsOff();
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
}