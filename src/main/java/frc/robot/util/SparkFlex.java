package frc.robot.util;

import com.revrobotics.CANSparkBase;
import com.revrobotics.CANSparkFlex;
import com.revrobotics.REVLibError;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import frc.robot.util.constants.LogConstants;

import java.util.function.Supplier;

//TODO: This needs to be merged with SparkMax class, they are are fully identical except for the alt encoder which we can write a check for.
public class SparkFlex extends CANSparkFlex {

  public enum MotorModel {
    VORTEX,
    NEO,
    NEO_550,
    BRUSHED
  }

  private static final int MAX_RETRIES = 5;

  private static final double INITIAL_RETRY_DELAY = 0.3; // Initial delay in seconds
  private static final double MAX_RETRY_DELAY = 2.0; // Maximum delay in seconds
  private static final double BACKOFF_MULTIPLIER = 2.0; // Multiplier for each retry

  private boolean initialized = false;
  private boolean deviceConnectionStatus = false;

  public SparkFlex(int canBusId) {
    super(canBusId, MotorType.kBrushless);

    // Check if the device is connected
    if (isDeviceConnected()) {

      // Restore factory defaults
      restoreFactoryDefaults();

      // Enable voltage compensation to 12V
      enableVoltageCompensation(12.0);
    } else {
      String errorMessage = String.format("CANSparkFlex (%s): Device not connected, skipping", canBusId);
      DataTracker.putString(LogConstants.ROBOT_SYSTEM, "SparkFlex", errorMessage, false);
      DriverStation.reportError(errorMessage, false);
    }
  }

  // #region CANSparkLowLevel overrides

  /// Note: The following methods are overrides of the CANSparkLowLevel methods.
  /// Should likely add all of them, but only adding the ones we need for now.

  @Override
  public REVLibError restoreFactoryDefaults() {
    return applyCommandWithRetry(super::restoreFactoryDefaults, "restoreFactoryDefaults");
  }

  // #endregion CANSparkLowLevel overrides

  // #region CANSparkBase overrides

  @Override
  public REVLibError setSmartCurrentLimit(int limit) {
    return applyCommandWithRetry(() -> super.setSmartCurrentLimit(limit), "setSmartCurrentLimit");
  }

  @Override
  public REVLibError setSmartCurrentLimit(int stallLimit, int freeLimit) {
    return applyCommandWithRetry(() -> super.setSmartCurrentLimit(stallLimit, freeLimit), "setSmartCurrentLimit");
  }

  @Override
  public REVLibError setSmartCurrentLimit(int stallLimit, int freeLimit, int limitRPM) {
    return applyCommandWithRetry(() -> super.setSmartCurrentLimit(stallLimit, freeLimit, limitRPM), "setSmartCurrentLimit");
  }

  @Override
  public REVLibError setSecondaryCurrentLimit(double limit) {
    return applyCommandWithRetry(() -> super.setSecondaryCurrentLimit(limit), "setSecondaryCurrentLimit");
  }

  @Override
  public REVLibError setSecondaryCurrentLimit(double limit, int chopCycles) {
    return applyCommandWithRetry(() -> super.setSecondaryCurrentLimit(limit, chopCycles), "setSecondaryCurrentLimit");
  }

  @Override
  public REVLibError setIdleMode(IdleMode mode) {
    return applyCommandWithRetry(() -> super.setIdleMode(mode), "setIdleMode");
  }

  @Override
  public REVLibError enableVoltageCompensation(double nominalVoltage) {
    return applyCommandWithRetry(() -> super.enableVoltageCompensation(nominalVoltage), "enableVoltageCompensation");
  }

  @Override
  public REVLibError disableVoltageCompensation() {
    return applyCommandWithRetry(super::disableVoltageCompensation, "disableVoltageCompensation");
  }

  @Override
  public REVLibError setOpenLoopRampRate(double rate) {
    return applyCommandWithRetry(() -> super.setOpenLoopRampRate(rate), "setOpenLoopRampRate");
  }

  @Override
  public REVLibError setClosedLoopRampRate(double rate) {
    return applyCommandWithRetry(() -> super.setClosedLoopRampRate(rate), "setClosedLoopRampRate");
  }

  @Override
  public REVLibError follow(final CANSparkBase leader) {
    return applyCommandWithRetry(() -> super.follow(leader), "follow");
  }

  @Override
  public REVLibError follow(final CANSparkBase leader, boolean invert) {
    return applyCommandWithRetry(() -> super.follow(leader, invert), "follow");
  }

  @Override
  public REVLibError follow(ExternalFollower leader, int deviceID) {
    return applyCommandWithRetry(() -> super.follow(leader, deviceID), "follow");
  }

  @Override
  public REVLibError follow(ExternalFollower leader, int deviceID, boolean invert) {
    return applyCommandWithRetry(() -> super.follow(leader, deviceID, invert), "follow");
  }

  @Override
  public REVLibError clearFaults() {
    return applyCommandWithRetry(super::clearFaults, "clearFaults");
  }

  @Override
  public REVLibError burnFlash() {
    return applyCommandWithRetry(super::burnFlash, "burnFlash");
  }

  @Override
  public REVLibError setCANTimeout(int milliseconds) {
    return applyCommandWithRetry(() -> super.setCANTimeout(milliseconds), "setCANTimeout");
  }

  @Override
  public REVLibError enableSoftLimit(SoftLimitDirection direction, boolean enable) {
    return applyCommandWithRetry(() -> super.enableSoftLimit(direction, enable), "enableSoftLimit");
  }

  @Override
  public REVLibError setSoftLimit(SoftLimitDirection direction, float limit) {
    return applyCommandWithRetry(() -> super.setSoftLimit(direction, limit), "setSoftLimit");
  }

  @Override
  public REVLibError getLastError() {
    return applyCommandWithRetry(super::getLastError, "getLastError");
  }

  // #endregion CANSparkBase overrides

  /**
   * Executes a command with retry logic using exponential backoff. This is effective for transient errors.
   *
   * @param command    A {@link Supplier} of {@link REVLibError}, representing the command to execute.
   * @param methodName Name of the method for logging.
   * @return {@link REVLibError} status of the command. Returns success status on early success or the last error after max retries.
   * 
   *         The retry logic starts with an initial delay (INITIAL_RETRY_DELAY) and increases the delay after each failed attempt
   *         by a factor of BACKOFF_MULTIPLIER, capped at MAX_RETRY_DELAY. A warning is logged on each failed attempt. If all attempts fail,
   *         an error is logged.
   */
  private REVLibError applyCommandWithRetry(Supplier<REVLibError> command, String methodName) {
    // Check if the device is connected before proceeding
    if (!isDeviceConnected()) {
      String errorMessage = String.format("CANSparkFlex (%s): Device not connected, skipping %s", this.getDeviceId(), methodName);
      DataTracker.putString(LogConstants.ROBOT_SYSTEM, "SparkFlex", errorMessage, false);
      DriverStation.reportError(errorMessage, false);
      return REVLibError.kCANDisconnected;
    }

    REVLibError status = REVLibError.kUnknown;
    double currentDelay = INITIAL_RETRY_DELAY;

    for (int i = 0; i < MAX_RETRIES; i++) {
      status = command.get();
      if (status == REVLibError.kOk) {
        return status;
      }

      // Apply backoff strategy
      Timer.delay(currentDelay);
      currentDelay = Math.min(currentDelay * BACKOFF_MULTIPLIER, MAX_RETRY_DELAY);

      // Log retry attempt
      String retryLog = String.format("CANSparkMax (%s): %s attempt %d failed, retrying in %.2f seconds", this.getDeviceId(), methodName, i + 1, currentDelay);
      DriverStation.reportError(retryLog, false);
    }

    String error = String.format("CANSparkMax (%s): %s failed after %s attempts", this.getDeviceId(), methodName, MAX_RETRIES);
    DriverStation.reportError(error, false);
    return status;
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
      String firmwareVersion = getFirmwareString();

      // Check if firmwareVersion is not null and not empty
      deviceConnectionStatus = firmwareVersion != null && !firmwareVersion.isEmpty() && !firmwareVersion.equalsIgnoreCase("v0.0.0");
      initialized = true;
    }
    return deviceConnectionStatus;
  }
}