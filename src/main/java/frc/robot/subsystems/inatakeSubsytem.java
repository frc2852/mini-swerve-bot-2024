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

  private CANSparkFlex topRollerMoto;
  private CANSparkFlex bottomRollerMoto;

  private DigitalInput intakeLimitSwitch;

  private double wheelIntakeMaxSpeed = 1;
  private double rollerIntakeMaxSpeed = 1;

  public inatakeSubsytem() {
    topRollerMoto = new CANSparkFlex(10, MotorType.kBrushless);
    topRollerMoto.setInverted(false);
    topRollerMoto.setIdleMode(IdleMode.kCoast);
    topRollerMoto.burnFlash();

    bottomRollerMoto = new CANSparkFlex(11, MotorType.kBrushless);
    bottomRollerMoto.follow(topRollerMoto);
    bottomRollerMoto.setInverted(true);
    bottomRollerMoto.setIdleMode(IdleMode.kCoast);
    bottomRollerMoto.burnFlash();

    intakeLimitSwitch = new DigitalInput(4);

  }

  // copied this from the last code
  private void setMotorSpeed(double motorSpeedPercentage) {
    double motorSpeed = motorSpeedPercentage / 100;
    topRollerMoto.set(motorSpeed);
    bottomRollerMoto.set(motorSpeed);
  }

  @Override
  public void periodic() {
  }

  public void stopIntake() {
    topRollerMoto.set(0);
    bottomRollerMoto.set(0);

  }

  public void RunIntakeIn() {
    setMotorSpeed(25);
  }

  public void RunIntakeOut() {
    setMotorSpeed(-25);
  }

  public boolean isNoteIntakeComplete() {
    // != invert our value /or not equal to
    boolean val = intakeLimitSwitch.get();// false
    boolean returnVal = !val;// true
    return !intakeLimitSwitch.get();
  }

}
