package frc.robot.util;

import com.revrobotics.CANSparkBase;
import com.revrobotics.CANSparkMax;
import com.revrobotics.REVLibError;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;

import java.util.function.Supplier;

public class SparkMax extends CANSparkMax {

    public enum MotorModel {
        NEO,
        NEO_550,
        BRUSHED
    }

    private static final int MAX_RETRIES = 5;
    
    private static final double INITIAL_RETRY_DELAY = 0.3; // Initial delay in seconds
    private static final double MAX_RETRY_DELAY = 2.0; // Maximum delay in seconds
    private static final double BACKOFF_MULTIPLIER = 2.0; // Multiplier for each retry

    public SparkMax(int canBusId, MotorModel motorType) {
        super(canBusId, (motorType == MotorModel.NEO || motorType == MotorModel.NEO_550) ? MotorType.kBrushless : MotorType.kBrushed);

        // Restore factory defaults
        restoreFactoryDefaults();

        // Enable voltage compensation to 12V
        enableVoltageCompensation(12.0);

        // For NEO_550 motors, enable Smart Current Limit to 20 amps
        // This can be overridden by the caller if needed
        if (motorType == MotorModel.NEO_550) {
            setSmartCurrentLimit(20);
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
     * Applies a given command with retry logic. The method retries the command execution
     * up to a specified maximum number of times (MAX_RETRIES) with an increasing delay
     * between each attempt (exponential backoff strategy). This method is useful for
     * handling transient errors in command execution.
     *
     * @param command    The command to be executed. It is a {@link Supplier} that returns a
     *                   {@link REVLibError}, allowing for any command conforming to this
     *                   interface to be passed.
     * @param methodName The name of the method/command being executed. This is used for logging
     *                   purposes to provide clearer context in error messages.
     * @return The status of the command execution as a {@link REVLibError}. If the command
     *         is successful before exceeding the maximum retry count, the success status
     *         is returned. If all retries fail, the last error status is returned.
     *
     *         The method implements an exponential backoff strategy for retries. The initial delay
     *         before the first retry is defined by INITIAL_RETRY_DELAY, and it increases by a factor
     *         of BACKOFF_MULTIPLIER after each failed attempt. The delay is capped at MAX_RETRY_DELAY
     *         to prevent excessively long waiting periods. This strategy helps to handle scenarios
     *         where immediate retry might not be effective.
     *
     *         In each retry attempt, if the command fails, a warning is logged with the current retry
     *         count and the next retry delay. If all attempts fail, an error message is logged indicating
     *         the failure of the command after the maximum number of retries.
     */
    private REVLibError applyCommandWithRetry(Supplier<REVLibError> command, String methodName) {
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
}