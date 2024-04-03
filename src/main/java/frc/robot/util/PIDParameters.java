// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.util;

import com.revrobotics.SparkPIDController;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.constants.Constants;

public class PIDParameters {

  private String groupId;
  private String namePrefix;
  private double P;
  private double I;
  private double D;
  private double Iz; // Integral zone
  private double FF; // Feedforward
  private double MinOutput;
  private double MaxOutput;

  private boolean pendingPIDUpdate = false;

  // Getters
  public double getP() {
    return P;
  }

  public double getI() {
    return I;
  }

  public double getD() {
    return D;
  }

  public double getIz() {
    return Iz;
  }

  public double getFF() {
    return FF;
  }

  public double getMaxOutput() {
    return MaxOutput;
  }

  public double getMinOutput() {
    return MinOutput;
  }

  // Setters
  public void setP(double kP) {
    this.P = kP;
  }

  public void setI(double kI) {
    this.I = kI;
  }

  public void setKD(double kD) {
    this.D = kD;
  }

  public void setKIz(double kIz) {
    this.Iz = kIz;
  }

  public void setFF(double kFF) {
    this.FF = kFF;
  }

  public void setMaxOutput(double kMaxOutput) {
    this.MaxOutput = kMaxOutput;
  }

  public void setMinOutput(double kMinOutput) {
    this.MinOutput = kMinOutput;
  }

  public PIDParameters(String groupId, String namePrefix, double P, double I, double D, double Iz, double FF, double MinOutput, double MaxOutput) {
    this.groupId = groupId;
    this.namePrefix = namePrefix;
    this.P = P;
    this.I = I;
    this.D = D;
    this.Iz = Iz;
    this.FF = FF;
    this.MinOutput = MinOutput;
    this.MaxOutput = MaxOutput;

    displayParameters();
  }

  public boolean updateParametersFromDashboard() {
    if (DriverStation.isFMSAttached() || !Constants.PID_TUNE_MODE)
      return false;

    double newP = getNumber(groupId, namePrefix + "P", P);
    if (newP != P) {
      P = newP;
      pendingPIDUpdate = true;
    }

    double newI = getNumber(groupId, namePrefix + "I", I);
    if (newI != I) {
      I = newI;
      pendingPIDUpdate = true;
    }

    double newD = getNumber(groupId, namePrefix + "D", D);
    if (newD != D) {
      D = newD;
      pendingPIDUpdate = true;
    }

    double newIz = getNumber(groupId, namePrefix + "Iz", Iz);
    if (newIz != Iz) {
      Iz = newIz;
      pendingPIDUpdate = true;
    }

    double newFF = getNumber(groupId, namePrefix + "FF", FF);
    if (newFF != FF) {
      FF = newFF;
      pendingPIDUpdate = true;
    }

    double newMinOutput = getNumber(groupId, namePrefix + "MinOutput", MinOutput);
    if (newMinOutput != MinOutput) {
      MinOutput = newMinOutput;
      pendingPIDUpdate = true;
    }

    double newMaxOutput = getNumber(groupId, namePrefix + "MaxOutput", MaxOutput);
    if (newMaxOutput != MaxOutput) {
      MaxOutput = newMaxOutput;
      pendingPIDUpdate = true;
    }

    return pendingPIDUpdate;
  }

  public void applyParameters(SparkPIDController pidController) {
    pidController.setP(P);
    pidController.setI(I);
    pidController.setD(D);
    pidController.setIZone(Iz);
    pidController.setFF(FF);
    pidController.setOutputRange(MinOutput, MaxOutput);
    pendingPIDUpdate = false;
  }

  private void displayParameters() {
    if (DriverStation.isFMSAttached() || !Constants.PID_TUNE_MODE)
      return;

    DataTracker.putNumber(groupId, namePrefix + "P", P, true);
    DataTracker.putNumber(groupId, namePrefix + "I", I, true);
    DataTracker.putNumber(groupId, namePrefix + "D", D, true);
    DataTracker.putNumber(groupId, namePrefix + "Iz", Iz, true);
    DataTracker.putNumber(groupId, namePrefix + "FF", FF, true);
  }

  private double getNumber(String groupId, String key, double defaultValue) {
    return SmartDashboard.getNumber(groupId + key, MaxOutput);
  }
}
