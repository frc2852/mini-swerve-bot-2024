// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import static edu.wpi.first.units.MutableMeasure.mutable;
import static edu.wpi.first.units.Units.Rotations;
import static edu.wpi.first.units.Units.RotationsPerSecond;
import static edu.wpi.first.units.Units.Volts;

import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;

import edu.wpi.first.units.Angle;
import edu.wpi.first.units.Measure;
import edu.wpi.first.units.MutableMeasure;
import edu.wpi.first.units.Velocity;
import edu.wpi.first.units.Voltage;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.I2C;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.I2C.Port;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Config;
import frc.robot.Constants.CanbusId;
import frc.robot.Constants.DIOId;
import frc.robot.Constants.MotorSetpoint;
import frc.robot.util.DataTracker;
import frc.robot.util.PIDParameters;
import frc.robot.util.SparkFlex;

public class IntakeSubsystem extends SubsystemBase {

  private final SparkFlex topRollers = new SparkFlex(CanbusId.INTAKE_TOP_ROLLER);
  private final SparkPIDController topRollersPID = topRollers.getPIDController();
  private final RelativeEncoder topRollersEncoder = topRollers.getEncoder();
  private PIDParameters topRollersPidParameters;

  private final SparkFlex bottomRollers = new SparkFlex(CanbusId.INTAKE_BOTTOM_ROLLER);
  private final SparkPIDController bottomRollersPID = bottomRollers.getPIDController();
  private final RelativeEncoder bottomRollersEncoder = bottomRollers.getEncoder();
  private PIDParameters bottomRollersPidParameters;

  private final DigitalInput intakeProximitySensor;
  private I2C ledStrip;

  private boolean isIntakeRunning = false;
  private boolean isConveyorMode = false;
  private double velocitySetpoint;

  private final double INTAKE_STOPPED_VELOCITY = 0.0;

  public enum LEDMode {
    OFF,
    ON,
  }

  public IntakeSubsystem() {

    // Set motor controller configurations
    topRollers.setIdleMode(IdleMode.kBrake);
    topRollers.setInverted(false);

    bottomRollers.setIdleMode(IdleMode.kBrake);
    bottomRollers.setInverted(true);

    // Set encoder configurations
    topRollersEncoder.setPosition(0);
    bottomRollersEncoder.setPosition(0);

    // Initialize proximity sensors
    intakeProximitySensor = new DigitalInput(DIOId.INTAKE_PROXIMITY_SENSOR);

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

    ledStrip = new I2C(Port.kOnboard, 0x01);
    // SysId configurations
    // setSysIdRoutine(topRollers, "intake-top-rollers");
    // setSysIdRoutine(bottomRollers, "intake-bottom-rollers");
  }

  @Override
  public void periodic() {
    // Automatically stop intake if game piece is loaded and we are not in conveyor mode
    if (isGamePieceLoaded() && isIntakeRunning && !isConveyorMode) {
      isConveyorMode = true;
      stopIntake();
    }

    if (isGamePieceLoaded()) {
      setLEDMode(LEDMode.OFF);
    } else {
      setLEDMode(LEDMode.ON);
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
    velocitySetpoint = INTAKE_STOPPED_VELOCITY;
    topRollersPID.setReference(INTAKE_STOPPED_VELOCITY, CANSparkMax.ControlType.kVelocity);
    bottomRollersPID.setReference(INTAKE_STOPPED_VELOCITY, CANSparkMax.ControlType.kVelocity);
  }

  public boolean isGamePieceLoaded() {
    return !intakeProximitySensor.get();
  }

  private void UpdateDataTracking() {
    // Get current positions and calculate errors
    double topRollersVelocity = topRollersEncoder.getVelocity();
    double topRollersVelocityError = velocitySetpoint - topRollersVelocity;

    double bottomRollersVelocity = bottomRollersEncoder.getVelocity();
    double bottomRollersVelocityError = velocitySetpoint - bottomRollersVelocity;

    DataTracker.putBoolean(getName(), "IsRunning", isIntakeRunning, true);
    DataTracker.putBoolean(getName(), "IsGamePieceLoaded", isGamePieceLoaded(), true);

    DataTracker.putNumber(getName(), "VelocitySetPoint", velocitySetpoint, true);
    DataTracker.putNumber(getName(), "TopRollersVelocity", topRollersVelocity, true);
    DataTracker.putNumber(getName(), "TopRollersVelocityError", topRollersVelocityError, true);
    DataTracker.putNumber(getName(), "BottomRollersVelocity", bottomRollersVelocity, true);
    DataTracker.putNumber(getName(), "BottomRollersVelocityError", bottomRollersVelocityError, true);
  }

  // #region SysId

  // Mutable holder for unit-safe voltage and velocity values
  private SysIdRoutine intakeSysIdRoutine;
  private final MutableMeasure<Voltage> appliedVoltage = mutable(Volts.of(0));
  private final MutableMeasure<Angle> distance = mutable(Rotations.of(0));
  private final MutableMeasure<Velocity<Angle>> velocity = mutable(RotationsPerSecond.of(0));

  // Create a new SysId routine for characterizing the intake.
  private void setSysIdRoutine(SparkFlex motor, String motorName) {
    RelativeEncoder encoder = motor.getEncoder();
    intakeSysIdRoutine = new SysIdRoutine(new Config(), new SysIdRoutine.Mechanism(
        (Measure<Voltage> voltage) -> motor.set(voltage.in(Volts) / RobotController.getBatteryVoltage()),
        log -> {
          log.motor(motorName)
              .voltage(appliedVoltage.mut_replace(motor.get() * RobotController.getBatteryVoltage(), Volts))
              .angularPosition(distance.mut_replace(encoder.getPosition(), Rotations))
              .angularVelocity(velocity.mut_replace(encoder.getVelocity(), RotationsPerSecond));
        }, this));
  }

  public Command sysIdQuasistatic(SysIdRoutine.Direction direction) {
    if (intakeSysIdRoutine == null) {
      throw new IllegalStateException("SysId routine not initialized");
    }

    return intakeSysIdRoutine.quasistatic(direction);
  }

  public Command sysIdDynamic(SysIdRoutine.Direction direction) {
    return intakeSysIdRoutine.dynamic(direction);
  }

  private void setLEDMode(LEDMode mode) {
    byte[] message = new byte[1];
    message[0] = (byte) mode.ordinal(); // Convert the mode to a byte value
    byte[] messageRecieved = new byte[0];

    // Send the mode to both Arduinos
    if (ledStrip != null) {
      ledStrip.transaction(message, message.length, messageRecieved, 0);
    }
  }

  // #endregion
}
