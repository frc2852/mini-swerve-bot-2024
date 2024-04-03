// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkMax;

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
import frc.robot.util.vision.Color;

public class Shooter extends SubsystemBase {

  // Controllers
  private final SparkFlex topRoller;
  private final SparkPIDController topRollerPID;
  private final RelativeEncoder topRollerEncoder;
  private PIDParameters topRollerPidParameters;

  private final SparkFlex bottomRoller;
  private final SparkPIDController bottomRollerPID;
  private final RelativeEncoder bottomRollerEncoder;
  private PIDParameters bottomRollerPidParameters;

  // Sensors
  private final DigitalInput shooterBeamBreak;

  // State
  private double velocitySetpoint;
  private boolean hasGamePieceBeenShot = false;
  private boolean autoKeepFlywheelRunning = false;

  // Smartdashboard
  private boolean updateTopRollerPID = false;
  private boolean updateBottomRollerPID = false;

  private LEDs leds;

  public Shooter(Object... args) {

    // Initialize motor controllers
    topRoller = new SparkFlex(CanbusId.SHOOTER_TOP_ROLLER);
    topRoller.setIdleMode(IdleMode.kCoast);
    topRoller.setInverted(false);

    bottomRoller = new SparkFlex(CanbusId.SHOOTER_BOTTOM_ROLLER);
    bottomRoller.setIdleMode(IdleMode.kCoast);
    bottomRoller.setInverted(true);

    // Initialize sensors
    shooterBeamBreak = new DigitalInput(DIOId.SHOOTER_BEAM_BREAK);

    // Initialize PID controllers
    topRollerPID = topRoller.getPIDController();
    topRollerEncoder = topRoller.getEncoder();

    bottomRollerPID = bottomRoller.getPIDController();
    bottomRollerEncoder = bottomRoller.getEncoder();

    // PID coefficients
    topRollerPidParameters = new PIDParameters(
        getName(),
        "TopRoller",
        0.0001, 0.000001, 0, 0, 0, -1, 1);

    bottomRollerPidParameters = new PIDParameters(
        getName(),
        "BottomRoller",
        0.0001, 0.000001, 0, 0, 0, -1, 1);

    // set PID coefficients
    topRollerPidParameters.applyParameters(topRollerPID);
    bottomRollerPidParameters.applyParameters(bottomRollerPID);

    // Save configuration to SparkMax flash
    topRoller.burnFlash();
    bottomRoller.burnFlash();

    // Add update buttons to dashboard
    if (!DriverStation.isFMSAttached() && Constants.PID_TUNE_MODE) {
      DataTracker.putBoolean(getName(), "UpdateTopRollerPID", updateTopRollerPID, true);
      DataTracker.putBoolean(getName(), "UpdateBottomRollerPID", updateBottomRollerPID, true);
    }

    leds = (LEDs) args[0];
  }

  @Override
  public void periodic() {

    // Allows us to lock the fly wheel at full speed to speed up auto
    if (autoKeepFlywheelRunning) {

      // If the path ran too long and didn't stop the flywheel, stop it
      if (DriverStation.isTeleop()) {
        autoKeepFlywheelRunning = false;
        stopShooter();
      } else {
        flyWheelFullSpeed();
      }
    }

    // If the game piece has been shot, reset the state
    if (!hasGamePieceBeenShot && isGamePieceDetected()) {
      hasGamePieceBeenShot = true;
    }

    if (isShooterRunning()) {
      leds.setLEDColor(Color.RED);
    }

    if (!DriverStation.isFMSAttached()) {
      // Get current positions and calculate errors
      double topRollerVelocity = topRollerEncoder.getVelocity();
      double topRollerVelocityError = velocitySetpoint == 0 ? 0 : topRollerVelocity - velocitySetpoint;

      double bottomRollerVelocity = bottomRollerEncoder.getVelocity();
      double bottomRollerVelocityError = velocitySetpoint == 0 ? 0 : topRollerVelocity - velocitySetpoint;

      // Dashboard data tracking
      DataTracker.putBoolean(getName(), "HasGamePieceBeenShot", hasGamePieceBeenShot, true);

      DataTracker.putNumber(getName(), "VelocitySetPoint", velocitySetpoint, true);

      DataTracker.putNumber(getName(), "TopRollerVelocity", topRollerVelocity, true);
      DataTracker.putNumber(getName(), "TopRollerVelocityError", topRollerVelocityError, true);

      DataTracker.putNumber(getName(), "BottomRollerVelocity", bottomRollerVelocity, true);
      DataTracker.putNumber(getName(), "BottomRollerVelocityError", bottomRollerVelocityError, true);

      if (Constants.PID_TUNE_MODE) {
        // PID updates from dashboard
        updateTopRollerPID = SmartDashboard.getBoolean("UpdateTopRollerPID", false);
        updateBottomRollerPID = SmartDashboard.getBoolean("UpdateBottomRollerPID", false);

        if (topRollerPidParameters.updateParametersFromDashboard() && updateTopRollerPID) {
          updateTopRollerPID = false;
          topRollerPidParameters.applyParameters(topRollerPID);
        }

        if (bottomRollerPidParameters.updateParametersFromDashboard() && updateBottomRollerPID) {
          updateBottomRollerPID = false;
          bottomRollerPidParameters.applyParameters(bottomRollerPID);
        }
      }
    }
  }

  public void divertGamePiece() {
    setShooterSpeed(MotorSetpoint.SHOOTER_DIVERT_VELOCITY, true);
  }

  public void flyWheelHalfSpeed() {
    setShooterSpeed(MotorSetpoint.SHOOTER_HALF_VELOCITY, false);
  }

  public void flyWheelFullSpeed() {
    setShooterSpeed(MotorSetpoint.SHOOTER_VELOCITY, false);
  }

  // Only call this from auto, this will lock the flywheel on
  public void autoShooterStart() {
    autoKeepFlywheelRunning = true;
    flyWheelFullSpeed();
  }

  // Only call this from auto
  public void autoShooterStop() {
    autoKeepFlywheelRunning = false;
    stopShooter();
  }

  public void stopShooter() {
    velocitySetpoint = 0;
    hasGamePieceBeenShot = false;
    topRoller.stopMotor();
    bottomRoller.stopMotor();
  }

  public void resetState() {
    hasGamePieceBeenShot = false;
  }

  public boolean isShooterAtSpeed() {
    double topRollerVelocity = topRollerEncoder.getVelocity();
    double bottomRollerVelocity = bottomRollerEncoder.getVelocity();
    return (Math.abs(topRollerVelocity - velocitySetpoint) < MotorSetpoint.SHOOTER_MARGIN_OF_ERROR && Math.abs(bottomRollerVelocity - velocitySetpoint) < MotorSetpoint.SHOOTER_MARGIN_OF_ERROR);
  }

  public boolean isShooterRunning() {
    return velocitySetpoint > 0;
  }

  public boolean hasGamePieceBeenShot() {
    return hasGamePieceBeenShot;
  }

  private void setShooterSpeed(double velocity, boolean isAmpShot) {
    velocitySetpoint = velocity;
    topRollerPID.setReference(isAmpShot ? 0 : velocitySetpoint, CANSparkMax.ControlType.kVelocity);
    bottomRollerPID.setReference(isAmpShot ? -velocitySetpoint : velocitySetpoint, CANSparkMax.ControlType.kVelocity);
  }

  private boolean isGamePieceDetected() {
    return !shooterBeamBreak.get();
  }
}
