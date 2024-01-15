// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkFlex;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.CommandPS4Controller;

public class inatakeSubsytem extends SubsystemBase {
  
  private CANSparkFlex topRollerMoto;
  private CANSparkFlex bottomRollerMoto;
  private CANSparkFlex topShooter;
  private CANSparkFlex bottomShooter;

  //is this part of the intake?
  private CANSparkFlex topBeltsConveryorMoto;
  private CANSparkFlex bottomBeltsConveryorMoto;
  private CANSparkFlex leftWinch;
  private CANSparkFlex rightWinch;
  //...

  private DigitalInput intakeLimitSwitch; 

  private double wheelIntakeMaxSpeed =1;
  private double rollerIntakeMaxSpeed =1;

  private double wheelShooterMaxSpeed =2;
  private double rollShooterMaxSpeed =2;

  private double stopIntake (0); //why is zero a problem

  public inatakeSubsytem() {
    topRollerMoto = new CANSparkFlex( 10,  MotorType.kBrushless);
    topRollerMoto.setInverted(false);
    topRollerMoto.setIdleMode(IdleMode.kCoast);
    topRollerMoto.burnFlash();

    bottomRollerMoto = new CANSparkFlex( 11,  MotorType.kBrushless);
    bottomRollerMoto.follow(topRollerMoto); 
    bottomRollerMoto.setInverted(true);
    bottomRollerMoto.setIdleMode(IdleMode.kCoast);
    bottomRollerMoto.burnFlash();

    topShooter = new CANSparkFlex( 12,  MotorType.kBrushless);
    topShooter.setInverted(false);
    topShooter.setIdleMode(IdleMode.kCoast);
    topShooter.burnFlash();

    bottomShooter = new CANSparkFlex( 13,  MotorType.kBrushless);
    bottomShooter.setInverted(true);
    bottomShooter.setIdleMode(IdleMode.kCoast);
    bottomShooter.burnFlash();

    intakeLimitSwitch= new DigitalInput(4);
    setMotorSpeed(80.0);
  }

  public void RunIntakeIn(){
   setMotorSpeed(25 );
  }
  public void RunIntakeOut(){
    setMotorSpeed(-25);
  }
  public void RunIntake(){
    setMotorSpeed(0);
  }

  public void RunShooterIn(){
   setMotorSpeed(35);
  }
  public void RunShooterOut(){
    setMotorSpeed(-35);
  }
  public void RunShooter(){
    setMotorSpeed(0);
  }

  //copied this from the last code 
  private void setMotorSpeed( double motorSpeedPercentage){
    double motorSpeed= motorSpeedPercentage /100;
    topRollerMoto.set(motorSpeed);
    bottomRollerMoto.set(motorSpeed);
  }

  private void setMotorSpeed( double motorSpeedPercentage){
    double motorSpeed= motorSpeedPercentage /100;
    topRollerMoto.set(motorSpeed);
    bottomRollerMoto.set(motorSpeed);
  }
  

  @Override
  public void periodic() {}
  public class RobotContainer{
  private final intakeSubSystem intakeSubSystem =new intakeSubSystem(); 
  //created a new class so it won't get ay error
  private final CommandPS4Controller driverController =new CommandPS4Controller(0);
  
  public RobotContainer(){
    configureBindings();
  }

  public void configureBindings(){
    driverController.L1().whileTrue(new RunCommand(()-> intakeSubSystem.RunIntakeIn()));
    driverController.R2().whileTrue(new RunCommand(()-> intakeSubSystem.RunIntakeOut()));
    //to solve this error I used the quick solve which is created a method for RunIntakeIn and RunIntakeOut
  }

public void runIntakeInwards(){
  if (isNoteIntakeComplete()== true){
    stopIntake();}else{
    topRollerMoto.set(wheelIntakeMaxSpeed);
    bottomRollerMoto.set(wheelIntakeMaxSpeed);
  }
}

private boolean isNoteIntakeComplete(){
  // != invert our value /or not equal to 
  boolean val= intakeLimitSwitch.get();//false
  boolean returnVal= !val;//true
  return!intakeLimitSwitch.get();
}

}
}
