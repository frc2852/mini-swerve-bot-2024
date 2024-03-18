package frc.robot.util;

import java.io.BufferedWriter;
import java.io.File;
import java.io.FileWriter;
import java.io.IOException;
import java.time.LocalDateTime;
import java.util.Random;
import java.util.concurrent.Executors;
import java.util.concurrent.ScheduledExecutorService;
import java.util.concurrent.ScheduledFuture;
import java.util.concurrent.TimeUnit;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.util.constants.LogConstants;

// TODO: Update to only log when robot is enabled. Massive logs are being generated when robot is left on filling up the usb drive.
// TODO: Add a check for the usb drive to be plugged in. If it is not, then do not log.
// TODO: Add a check for existing logs, only allow 1-2 logs to be saved at a time. If there are more, delete the oldest one.

// Temporarily disabled logging until the above TODOs are completed.

/**
 * This class provides logging functionalities for the FRC robot. It allows
 * logging of various types of data and manages file writing and smart dashboard
 * updates.
 */
public class DataTracker {

  private static DataTracker instance = new DataTracker();

  private BufferedWriter bufferedWriter;
  private StringBuilder logBuffer;
  private String loggingDirectory = "";

  private String fileName;
  private static final String LOG_FILE_PREFIX = "log_";
  private ScheduledExecutorService scheduler;

  private boolean fmsDetailsLogged = false;
  private boolean dsDetailsLogged = false;
  private ScheduledFuture<?> fmsCheckTaskFuture;
  private ScheduledFuture<?> dsCheckTaskFuture;

  /**
   * Initializes file writer, buffer, and tasks.
   */
  public DataTracker() {
    // initializeLoggingDirectory();
    // initializeLogFile();
    // startFlushTask();
    // startFmsCheckTask();
  }

  /**
   * Initializes the log file for writing.
   */
  private void initializeLogFile() {
    try {
      fileName = generateFileName();
      File file = new File(fileName);
      boolean isNewFile = file.createNewFile();

      bufferedWriter = new BufferedWriter(new FileWriter(file, true));
      logBuffer = new StringBuilder();
      scheduler = Executors.newSingleThreadScheduledExecutor();

      if (isNewFile) {
        writeHeaders();
      }
    } catch (IOException e) {
      DriverStation.reportError("Failed to initialize log file.", true);
    }
  }

  /**
   * Initializes the logging directory.
   */
  private void initializeLoggingDirectory() {
    if (RobotBase.isReal()) {
      File usb1 = new File("/media/sda1/");
      if (usb1.exists()) {
        loggingDirectory = "/media/sda1/logs/";
      } else {
        loggingDirectory = "/home/lvuser/logs";
      }
    }

    // Check if code is running in Simulation
    if (RobotBase.isSimulation()) {
      loggingDirectory = "./logs/";
    }
  }

  /**
   * Generates a unique file name for the log file.
   * 
   * @return A string representing the file name.
   */
  private String generateFileName() {
    Random random = new Random();
    int randomNumber = random.nextInt(90000000) + 10000000;
    return loggingDirectory + LOG_FILE_PREFIX + randomNumber + ".csv";
  }

  /**
   * Writes headers to the log file.
   */
  private void writeHeaders() {
    try {
      String headers = "GroupId,Key,Value,TimeStamp\n";
      bufferedWriter.write(headers);
      bufferedWriter.flush();
    } catch (Exception e) {
      DriverStation.reportError("Failed to write headers to log file.", true);
    }
  }

  /**
   * Writes a log entry to the log buffer.
   * 
   * @param groupId              The group ID of the log entry.
   * @param key                  The key of the log entry.
   * @param value                The value of the log entry.
   * @param sendToSmartDashboard Whether to send this entry to SmartDashboard.
   */
  private synchronized void writeToLog(String groupId, String key, String value, boolean sendToSmartDashboard) {
    try {
      String timeStamp = LocalDateTime.now().toString();
      String logMessage = String.format("%s,%s,%s,%s\n", groupId, key, value, timeStamp);
      logBuffer.append(logMessage);
    } catch (Exception e) {
      DriverStation.reportError("Failed to write to log file.", true);
    }
  }

  /**
   * Starts a task that flushes the log buffer to file periodically.
   */
  private void startFlushTask() {
    Runnable flushTask = () -> {
      try {
        synchronized (this) {
          bufferedWriter.write(logBuffer.toString());
          bufferedWriter.flush();
          logBuffer.setLength(0);
        }
      } catch (IOException e) {
        DriverStation.reportError("Failed to flush log buffer.", true);
      }
    };

    scheduler.scheduleAtFixedRate(flushTask, 1, 1, TimeUnit.SECONDS);
  }

  /**
   * Starts a task to check for FMS connection and log match details.
   */
  private void startFmsCheckTask() {
    Runnable fmsCheckTask = () -> {
      if (!fmsDetailsLogged && DriverStation.isFMSAttached()) {
        logFmsDetails();
      }
    };

    Runnable dsCheckTask = () -> {
      if (!dsDetailsLogged && DriverStation.isDSAttached()) {
        logStartUp();
      }
    };

    fmsCheckTaskFuture = scheduler.scheduleAtFixedRate(fmsCheckTask, 0, 1, TimeUnit.SECONDS);
    dsCheckTaskFuture = scheduler.scheduleAtFixedRate(dsCheckTask, 0, 1, TimeUnit.SECONDS);
  }

  /**
   * Logs FMS details to the log file.
   */
  private void logFmsDetails() {
    try {
      writeToLog(LogConstants.ROBOT_SYSTEM, "EventName", DriverStation.getEventName(), false);
      writeToLog(LogConstants.ROBOT_SYSTEM, "MatchType", DriverStation.getMatchType().name(), false);
      writeToLog(LogConstants.ROBOT_SYSTEM, "MatchNumber", String.valueOf(DriverStation.getMatchNumber()), false);
      fmsDetailsLogged = true;
      if (fmsCheckTaskFuture != null && !fmsCheckTaskFuture.isCancelled()) {
        fmsCheckTaskFuture.cancel(false);
      }
    } catch (Exception e) {
      DriverStation.reportError("Failed to log FMS details.", true);
    }
  }

  /**
   * Logs Match details to the log file.
   */
  private void logStartUp() {
    try {
      String timeStamp = LocalDateTime.now().toString();
      writeToLog(LogConstants.ROBOT_SYSTEM, "StartTime", timeStamp, false);
      dsDetailsLogged = true;
      if (dsCheckTaskFuture != null && !dsCheckTaskFuture.isCancelled()) {
        dsCheckTaskFuture.cancel(false);
      }
    } catch (Exception e) {
      DriverStation.reportError("Failed to log startup details.", true);
    }
  }

  /**
   * Returns the singleton instance of DataTracker.
   *
   * @return The singleton instance of DataTracker.
   */
  private static DataTracker getInstance() {
    return instance;
  }

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
    // DataTracker instance = getInstance();
    // instance.writeToLog(groupId, key, String.valueOf(value), sendToSmartDashboard);
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
    // DataTracker instance = getInstance();
    // instance.writeToLog(groupId, key, String.valueOf(value), sendToSmartDashboard);
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
    // DataTracker instance = getInstance();
    // instance.writeToLog(groupId, key, value, sendToSmartDashboard);
    if (sendToSmartDashboard) {
      SmartDashboard.putString(groupId + key, value);
    }
  }
}
