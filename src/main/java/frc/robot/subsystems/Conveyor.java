// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.Constants;
import frc.robot.constants.Constants.CanbusId;
import frc.robot.constants.Constants.DIOId;
import frc.robot.constants.Constants.MotorSetpoint;
import frc.robot.util.DataTracker;
import frc.robot.util.PIDParameters;
import frc.robot.util.SparkFlex;

public class Conveyor extends SubsystemBase {

  // Controllers
  private final SparkFlex topConveyor;
  private final SparkPIDController topConveyorPID;
  private final RelativeEncoder topConveyorEncoder;
  private PIDParameters topConveyorPidParameters;

  private final SparkFlex bottomConveyor;
  private final SparkPIDController bottomConveyorPID;
  private final RelativeEncoder bottomConveyorEncoder;
  private PIDParameters bottomConveyorPidParameters;

  // Sensors
  private final DigitalInput ampBeamBreak;

  // State
  private double velocitySetpoint;

  // Smartdashboard
  private boolean updateTopConveyorPID = false;
  private boolean updateBottomConveyorPID = false;

  public Conveyor() {
    // Initialize motor controllers
    topConveyor = new SparkFlex(CanbusId.CONVEYOR_TOP);
    topConveyor.setIdleMode(IdleMode.kCoast);
    topConveyor.setInverted(false);

    bottomConveyor = new SparkFlex(CanbusId.CONVEYOR_BOTTOM);
    bottomConveyor.setIdleMode(IdleMode.kCoast);
    bottomConveyor.setInverted(true);

    // Initialize sensors
    ampBeamBreak = new DigitalInput(DIOId.CONVEYOR_BEAM_BREAK);

    // Initialize PID controllers
    topConveyorPID = topConveyor.getPIDController();
    topConveyorEncoder = topConveyor.getEncoder();

    bottomConveyorPID = bottomConveyor.getPIDController();
    bottomConveyorEncoder = bottomConveyor.getEncoder();

    // PID coefficients
    topConveyorPidParameters = new PIDParameters(
        getName(),
        "TopMotor",
        0.0001, 0.000001, 0, 0, 0, -1, 1);

    bottomConveyorPidParameters = new PIDParameters(
        getName(),
        "BottomMotor",
        0.0001, 0.000001, 0, 0, 0, -1, 1);

    // set PID coefficients
    topConveyorPidParameters.applyParameters(topConveyorPID);
    bottomConveyorPidParameters.applyParameters(bottomConveyorPID);

    // Save configuration to SparkMax flash
    topConveyor.burnFlash();
    bottomConveyor.burnFlash();

    // Add update buttons to dashboard
    if (!DriverStation.isFMSAttached() && Constants.PID_TUNE_MODE) {
      DataTracker.putBoolean(getName(), "UpdateTopMotorPID", updateTopConveyorPID, true);
      DataTracker.putBoolean(getName(), "UpdateBottomMotorPID", updateBottomConveyorPID, true);
    }
  }

  @Override
  public void periodic() {
    if (!DriverStation.isFMSAttached()) {
      // Get current velocities of the conveyors
      double topConveyorVelocity = topConveyorEncoder.getVelocity();
      double bottomConveyorVelocity = bottomConveyorEncoder.getVelocity();

      // Calculate the velocity errors
      double topConveyorVelocityError = velocitySetpoint == 0 ? 0 : topConveyorVelocity - velocitySetpoint;
      double bottomConveyorVelocityError = velocitySetpoint == 0 ? 0 : bottomConveyorVelocity - velocitySetpoint;

      DataTracker.putNumber(getName(), "VelocitySetPoint", velocitySetpoint, true);
      DataTracker.putNumber(getName(), "TopVelocity", topConveyorVelocity, true);
      DataTracker.putNumber(getName(), "TopVelocityError", topConveyorVelocityError, true);
      DataTracker.putNumber(getName(), "BottomVelocity", bottomConveyorVelocity, true);
      DataTracker.putNumber(getName(), "BottomVelocityError", bottomConveyorVelocityError, true);
      DataTracker.putBoolean(getName(), "IsGamePieceAmpReady", isGamePieceAmpReady(), true);

      if (Constants.PID_TUNE_MODE) {
        // PID updates from dashboard
        updateTopConveyorPID = SmartDashboard.getBoolean("UpdateTopMotorPID", false);
        updateBottomConveyorPID = SmartDashboard.getBoolean("UpdateBottomMotorPID", false);

        if (topConveyorPidParameters.updateParametersFromDashboard() && updateTopConveyorPID) {
          updateTopConveyorPID = false;
          topConveyorPidParameters.applyParameters(topConveyorPID);
        }

        if (bottomConveyorPidParameters.updateParametersFromDashboard() && updateBottomConveyorPID) {
          updateBottomConveyorPID = false;
          bottomConveyorPidParameters.applyParameters(bottomConveyorPID);
        }
      }
    }
  }

  public void runConveyorForward() {
    setConveyorVelocity(MotorSetpoint.CONVEYOR_VELOCITY);
  }

  public void runConveyorReversed() {
    setConveyorVelocity(-MotorSetpoint.CONVEYOR_VELOCITY);
  }

  public void stopConveyor() {
    velocitySetpoint = 0;
    topConveyor.stopMotor();
    bottomConveyor.stopMotor();
  }

  public void runConveyorForwardAmp() {
    setConveyorVelocity(1000);
  }

  public boolean isGamePieceAmpReady() {
    return !ampBeamBreak.get();
  }

  private void setConveyorVelocity(double velocity) {
    velocitySetpoint = velocity;
    topConveyorPID.setReference(velocity, CANSparkMax.ControlType.kVelocity);
    bottomConveyorPID.setReference(velocity, CANSparkMax.ControlType.kVelocity);
  }
}