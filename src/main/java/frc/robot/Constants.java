// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide
 * numerical or boolean
 * constants. This class should not be used for any other purpose. All constants
 * should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>
 * It is advised to statically import this class (or one of its inner classes)
 * wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {

  // Setting true will enable SmartDashboard PID tuning
  public static boolean PID_TUNE_MODE = false;

  public static class OperatorConstant {
    public static final int DRIVER_CONTROLLER_PORT = 0;
    public static final int OPERATOR_CONTROLLER_PORT = 1;
    public static final int SYSID_CONTROLLER_PORT = 2;

    public static final double DEAD_BAND = 0.15;
    public static final double EXPONENTIAL_RESPONSE = 2;
  }

  public static class CanbusId {
    public static final int POWER_DISTRIBUTION_HUB = 1;
    public static final int FRONT_LEFT_DRIVE = 2, FRONT_LEFT_TURNING = 3;
    public static final int FRONT_RIGHT_DRIVE = 4, FRONT_RIGHT_TURNING = 5;
    public static final int REAR_LEFT_DRIVE = 6, REAR_LEFT_TURNING = 7;
    public static final int REAR_RIGHT_DRIVE = 8, REAR_RIGHT_TURNING = 9;

    public static final int INTAKE_TOP_ROLLER = 10;
    public static final int INTAKE_BOTTOM_ROLLER = 11;

    public static final int SHOOTER_TOP_ROLLER = 12;
    public static final int SHOOTER_BOTTOM_ROLLER = 13;

    public static final int CONVEYOR_TOP = 14;
    public static final int CONVEYOR_BOTTOM = 15;

    public static final int WINCH_LEFT = 16;
    public static final int WINCH_RIGHT = 17;

    public static final int ELEVATOR = 18;
    public static final int CLIMB_WHEELS = 19;
  }

  public static class DIOId {
    public static final int INTAKE_PROXIMITY_SENSOR = 0;
    public static final int SHOOTER_PROXIMITY_SENSOR = 1;
    public static final int CONVEYOR_PROXIMITY_SENSOR = 2;
  }

  // JOHN LOOK HERE //Hi Krystian!!! Hi guys!! - Liam <3
  public static class MotorSetpoint {
    // make 5000
    public static final int INTAKE_VELOCITY = 5000;
    public static final int CONVEYOR_VELOCITY = 2000;

    public static final int SHOOTER_MARGIN_OF_ERROR = 5;
    public static final int SHOOTER_DIVERT_VELOCITY = 1500;
    public static final int SHOOTER_VELOCITY = 3000;

    public static final int ELEVATOR_MARGIN_OF_ERROR = 2;
    public static final int ELEVATOR_DRIVE_POSITION = 0;
    public static final int ELEVATOR_AMP_POSITION = 70; // 220 60:1
    public static final int ELEVATOR_TRAP_POSITION = 100;

    // Value is between 0 and 1. 0.1 = 10% output
    public static final double ELEVAOTOR_MAX_OUPUT = 1.0;


    // Value is between 0 and 1. 0.1 = 10% output
    public static final double WINCH_MAX_OUPUT = 0.1;
  }

  public static class LEDConstants {
    // I2C Addresses for the Arduino controllers
    public static final int ARDUINO_ADDRESS = 0x01;
  }
}
