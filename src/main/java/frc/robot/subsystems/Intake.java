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
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.Constants.CanbusId;
import frc.robot.constants.Constants.DIOId;
import frc.robot.constants.Constants.MotorSetpoint;
import frc.robot.util.DataTracker;
import frc.robot.util.PIDParameters;
import frc.robot.util.SparkFlex;
import frc.robot.util.vision.Color;

public class Intake extends SubsystemBase {

  // Subsystems
  private final LEDs leds;
  private final PowerHub powerHub;

  // Controllers
  private final SparkFlex topRollers = new SparkFlex(CanbusId.INTAKE_TOP_ROLLER);
  private final SparkPIDController topRollersPID = topRollers.getPIDController();
  private final RelativeEncoder topRollersEncoder = topRollers.getEncoder();
  private PIDParameters topRollersPidParameters;

  private final SparkFlex bottomRollers = new SparkFlex(CanbusId.INTAKE_BOTTOM_ROLLER);
  private final SparkPIDController bottomRollersPID = bottomRollers.getPIDController();
  private final RelativeEncoder bottomRollersEncoder = bottomRollers.getEncoder();
  private PIDParameters bottomRollersPidParameters;

  // Sensors
  private final DigitalInput intakeBeamBreak;

  // State
  private boolean isIntakeRunning = false;
  private boolean isConveyorMode = false;
  private double velocitySetpoint;

  public Intake(Object... args) {

    // Set motor controller configurations
    topRollers.setIdleMode(IdleMode.kBrake);
    topRollers.setInverted(false);

    bottomRollers.setIdleMode(IdleMode.kBrake);
    bottomRollers.setInverted(true);

    // Set encoder configurations
    topRollersEncoder.setPosition(0);
    bottomRollersEncoder.setPosition(0);

    // Initialize sensors
    intakeBeamBreak = new DigitalInput(DIOId.INTAKE_BEAM_BREAK);

    // PID coefficients
    topRollersPidParameters = new PIDParameters(
        getName(),
        "TopRollers",
        0.0001, 0.000001, 0, 0, 0, -1, 1);

    bottomRollersPidParameters = new PIDParameters(
        getName(),
        "BottomRollers",
        0.0001, 0.000001, 0, 0, 0, -1, 1);

    // Set PID coefficients
    topRollersPidParameters.applyParameters(topRollersPID);
    bottomRollersPidParameters.applyParameters(bottomRollersPID);

    // Save configuration to SparkMax flash
    topRollers.burnFlash();
    bottomRollers.burnFlash();

    // Subsystems
    leds = (LEDs) args[0];
    powerHub = (PowerHub) args[1];
  }

  @Override
  public void periodic() {
    if(isIntakeRunning){
      leds.setLEDColor(Color.BLUE);
    }

    if (isGamePieceLoaded()) {
      leds.setLEDColor(Color.GREEN);

      // Automatically stop intake if game piece is loaded and we are not in conveyor mode
      if (isIntakeRunning && !isConveyorMode) {
        isConveyorMode = true;
        stopIntake();
      }
    } else {
      leds.setLEDColor(Color.OFF);
    }

    UpdateDataTracking();
  }

  public void toggleIntake() {
    isIntakeRunning = !isIntakeRunning;
    if (isIntakeRunning) {
      runIntake(false);
    } else {
      stopIntake();
    }
  }

  public void runIntake(boolean conveyorMode) {
    isIntakeRunning = true;
    isConveyorMode = conveyorMode;

    velocitySetpoint = MotorSetpoint.INTAKE_VELOCITY;
    topRollersPID.setReference(velocitySetpoint, CANSparkMax.ControlType.kVelocity);
    bottomRollersPID.setReference(velocitySetpoint, CANSparkMax.ControlType.kVelocity);
  }

  public void stopIntake() {
    isIntakeRunning = false;

    velocitySetpoint = 0;
    topRollersPID.setReference(velocitySetpoint, CANSparkMax.ControlType.kVelocity);
    bottomRollersPID.setReference(velocitySetpoint, CANSparkMax.ControlType.kVelocity);
  }

  public boolean isGamePieceLoaded() {
    return !intakeBeamBreak.get();
  }

  private void UpdateDataTracking() {
    DataTracker.putBoolean(getName(), "IsIntakeRunning", isIntakeRunning, true);
    DataTracker.putBoolean(getName(), "IsGamePieceLoaded", isGamePieceLoaded(), true);

    if (!DriverStation.isFMSAttached()) {
      // Get current positions and calculate errors
      double topRollersVelocity = topRollersEncoder.getVelocity();
      double topRollersVelocityError = velocitySetpoint - topRollersVelocity;

      double bottomRollersVelocity = bottomRollersEncoder.getVelocity();
      double bottomRollersVelocityError = velocitySetpoint - bottomRollersVelocity;

      DataTracker.putNumber(getName(), "VelocitySetPoint", velocitySetpoint, true);
      DataTracker.putNumber(getName(), "TopRollersVelocity", topRollersVelocity, true);
      DataTracker.putNumber(getName(), "TopRollersVelocityError", topRollersVelocityError, true);
      DataTracker.putNumber(getName(), "BottomRollersVelocity", bottomRollersVelocity, true);
      DataTracker.putNumber(getName(), "BottomRollersVelocityError", bottomRollersVelocityError, true);
    }
  }
}
