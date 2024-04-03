// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.constants;

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
    public static final double EXPONENTIAL_RESPONSE = 3;
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

    public static final int ELEVATOR = 18;
  }

  public static class DIOId {
    public static final int INTAKE_BEAM_BREAK = 0;
    public static final int SHOOTER_BEAM_BREAK = 1;
    public static final int CONVEYOR_BEAM_BREAK = 2;
  }

  public static class MotorSetpoint {
    public static final int INTAKE_VELOCITY = 5000;
    public static final int CONVEYOR_VELOCITY = 6784;

    public static final int SHOOTER_MARGIN_OF_ERROR = 5;
    public static final int SHOOTER_DIVERT_VELOCITY = 6784;
    public static final int SHOOTER_HALF_VELOCITY = 1500;
    public static final int SHOOTER_VELOCITY = 3000;

    public static final int ELEVATOR_MARGIN_OF_ERROR = 2;
    public static final int ELEVATOR_DRIVE_POSITION = 0;
    public static final int ELEVATOR_AMP_POSITION = 70;

    
    public static final int ELEVATOR_CLIMB_DOWN_POSITION = 0;
    public static final int ELEVATOR_CLIMB_UP_POSITION = 70;

    // Value is between 0 and 1. 0.1 = 10% output
    public static final double ELEVAOTOR_MAX_OUPUT = 1.0;
  }

  public static class SubsystemEnable {
    public static final boolean INTAKE = true;
    public static final boolean SHOOTER = true;
    public static final boolean CONVEYOR = true;
    public static final boolean ELEVATOR = true;
  }
}
