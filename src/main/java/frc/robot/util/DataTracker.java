package frc.robot.util;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/**
 * This class provides logging functionalities for the FRC robot. It allows
 * logging of various types of data and manages file writing and smart dashboard
 * updates.
 */
public class DataTracker {

  /**
   * Logs a boolean value with specified group and key.
   *
   * @param groupId              The group ID for categorizing the log entry.
   * @param key                  The key identifying the log entry.
   * @param value                The boolean value to log.
   * @param sendToSmartDashboard If true, sends the value to SmartDashboard.
   * @return True if the value was successfully sent to SmartDashboard, otherwise false.
   */
  public static boolean putBoolean(String groupId, String key, boolean value, boolean sendToSmartDashboard) {
    if (sendToSmartDashboard) {
      return SmartDashboard.putBoolean(groupId + key, value);
    }
    return false;
  }

  /**
   * Logs a numeric value with specified group and key.
   *
   * @param groupId              The group ID for categorizing the log entry.
   * @param key                  The key identifying the log entry.
   * @param value                The numeric value to log.
   * @param sendToSmartDashboard If true, sends the value to SmartDashboard.
   */
  public static void putNumber(String groupId, String key, double value, boolean sendToSmartDashboard) {
    if (sendToSmartDashboard) {
      SmartDashboard.putNumber(groupId + key, value);
    }
  }

  /**
   * Logs a string value with specified group and key.
   *
   * @param groupId              The group ID for categorizing the log entry.
   * @param key                  The key identifying the log entry.
   * @param value                The string value to log.
   * @param sendToSmartDashboard If true, sends the value to SmartDashboard.
   */
  public static void putString(String groupId, String key, String value, boolean sendToSmartDashboard) {
    if (sendToSmartDashboard) {
      SmartDashboard.putString(groupId + key, value);
    }
  }
}
