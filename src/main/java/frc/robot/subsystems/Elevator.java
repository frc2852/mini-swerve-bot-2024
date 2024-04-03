// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkMax;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.Constants;
import frc.robot.constants.Constants.CanbusId;
import frc.robot.constants.Constants.MotorSetpoint;
import frc.robot.util.DataTracker;
import frc.robot.util.PIDParameters;
import frc.robot.util.SparkFlex;

public class Elevator extends SubsystemBase {

  // Controllers
  private final SparkFlex motor;
  private final SparkPIDController motorPID;
  private final RelativeEncoder motorEncoder;
  private PIDParameters motorPidParameters;

  // State
  private double positionSetpoint;

  // Smartdashboard
  private boolean updateMotorPID = false;

  public Elevator() {

    // Initialize motor controllers
    motor = new SparkFlex(CanbusId.ELEVATOR);
    motor.setIdleMode(IdleMode.kBrake);
    motor.setInverted(false);

    // Initialize PID controllers
    motorPID = motor.getPIDController();
    motorEncoder = motor.getEncoder();

    // Zero encoder position
    motorEncoder.setPosition(0);

    // PID coefficients
    motorPidParameters = new PIDParameters(
        getName(),
        "",
        0.1, 0, 0, 0, 0, -MotorSetpoint.ELEVAOTOR_MAX_OUPUT, MotorSetpoint.ELEVAOTOR_MAX_OUPUT);

    // set PID coefficients
    motorPidParameters.applyParameters(motorPID);

    // Save configuration to SparkMax flash
    motor.burnFlash();

    // Add update buttons to dashboard
    if (!DriverStation.isFMSAttached() && Constants.PID_TUNE_MODE) {
      DataTracker.putBoolean(getName(), "UpdatePID", updateMotorPID, true);
    }
  }

  @Override
  public void periodic() {
    if (!DriverStation.isFMSAttached()) {
      // Get current positions and calculate errors
      double motorPosition = motorEncoder.getPosition();
      double motorPositionError = positionSetpoint - motorPosition;

      // Dashboard data tracking
      DataTracker.putNumber(getName(), "PositionSetPoint", positionSetpoint, true);
      DataTracker.putNumber(getName(), "Position", motorPosition, true);
      DataTracker.putNumber(getName(), "PositionError", motorPositionError, true);
      DataTracker.putBoolean(getName(), "helloWorld", isElevatorAtPosition(), true);

      if (Constants.PID_TUNE_MODE) {
        // PID updates from dashboard
        updateMotorPID = SmartDashboard.getBoolean("UpdatePID", false);

        if (motorPidParameters.updateParametersFromDashboard() && updateMotorPID) {
          updateMotorPID = false;
          motorPidParameters.applyParameters(motorPID);
        }
      }
    }
  }

  public void drivePosition() {
    setElevatorPosition(MotorSetpoint.ELEVATOR_DRIVE_POSITION);
  }

  public void ampPosition() {
    setElevatorPosition(MotorSetpoint.ELEVATOR_AMP_POSITION);
  }

  public void trapPosition() {
    setElevatorPosition(MotorSetpoint.ELEVATOR_TRAP_POSITION);
  }

  public boolean isElevatorAtPosition() {
    return Math.abs(motorEncoder.getPosition() - positionSetpoint) < MotorSetpoint.ELEVATOR_MARGIN_OF_ERROR;
  }

  private void setElevatorPosition(double position) {
    positionSetpoint = position;
    motorPID.setReference(position, CANSparkMax.ControlType.kPosition);
  }
}