// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkFlex;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class inatakeSubsytem extends SubsystemBase {

  private CANSparkFlex topRoller;
  private CANSparkFlex bottomRoller;

  private DigitalInput intakeLimitSwitch;
  private double rollerIntakeMaxSpeed = 1;

  public inatakeSubsytem() {
    topRoller = new CANSparkFlex(10, MotorType.kBrushless);
    topRoller.setInverted(false);
    topRoller.setIdleMode(IdleMode.kCoast);
    topRoller.burnFlash();

    bottomRoller = new CANSparkFlex(11, MotorType.kBrushless);
    bottomRoller.setInverted(true);
    bottomRoller.setIdleMode(IdleMode.kCoast);
    bottomRoller.burnFlash();

    intakeLimitSwitch = new DigitalInput(4);

  }

  // copied this from the last code
  private void setMotorSpeed(double motorSpeedPercentage) {
    double motorSpeed = motorSpeedPercentage / 100;
    topRoller.set(motorSpeed);
    bottomRoller.set(motorSpeed);
  }

  @Override
  public void periodic() {
  }

  public void stopIntake() {
    topRoller.set(0);
    bottomRoller.set(0);

  }

  public void RunIrollerIntakeMaxSpeedntakeIn() {
    setMotorSpeed(rollerIntakeMaxSpeed);
  }

  public void RunIrollerIntakeMaxSpeedntakeOut() {
    setMotorSpeed(-rollerIntakeMaxSpeed);
  }

  public boolean isNoteIntakeComplete() {
    // != invert our value /or not equal to
    boolean val = intakeLimitSwitch.get();// false
    boolean returnVal = !val;// true
    return !intakeLimitSwitch.get();
  }

}
