// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants.PowerDistributionConstants;

public class PowerDistributionSubsystem {
  private PowerDistribution powerDistribution;

  public PowerDistributionSubsystem() {
    powerDistribution = new PowerDistribution();
    SmartDashboard.putNumber("intakeCurrent", 0);
    SmartDashboard.putNumber("hookLeftCurrent", 0);
    SmartDashboard.putNumber("hookRightCurrent", 0);
    SmartDashboard.putNumber("upShooterCurrent", 0);
    SmartDashboard.putNumber("downShooterCurrent", 0);
    SmartDashboard.putNumber("lineCurrent", 0);
    SmartDashboard.putNumber("transportCurrent", 0);
    SmartDashboard.putNumber("rotateShooterCurrent", 0);
    SmartDashboard.putBoolean("isIntakeOverCurren", false);
    SmartDashboard.putBoolean("isHookLeftOverCurrent", false);
    SmartDashboard.putBoolean("isHookRightOverCurrent", false);
    SmartDashboard.putBoolean("isShooterUpOverCurren", false);
    SmartDashboard.putBoolean("isShooterDownOverCurrent", false);
    SmartDashboard.putBoolean("isTransportOverCurrent", false);
    SmartDashboard.putBoolean("isLineMoterOverCurrent", false);
    SmartDashboard.putBoolean("isRotateShooterOverCurrent", false);
  }

  public double intakeCurrent() {
    double current = powerDistribution.getCurrent(PowerDistributionConstants.kIntakeMotorCurrrentchannel);
    SmartDashboard.putNumber("intakeCurrent", current);
    return current;
  }

  public double hookLeftCurrent() {
    double current = powerDistribution.getCurrent(PowerDistributionConstants.kHookLeftMotorCurrentchannel);
    SmartDashboard.putNumber("hookLeftCurrent", current);
    return current;
  }

  public double hookRightCurrent() {
    double current = powerDistribution.getCurrent(PowerDistributionConstants.kHookRightMotorCurrentchannel);
    SmartDashboard.putNumber("hookRightCurrent", current);
    return current;
  }

  public double upShooterCurrent() {
    double current = powerDistribution.getCurrent(PowerDistributionConstants.kShooterUpMotorCurrentchannel);
    SmartDashboard.putNumber("upShooterCurrent", current);
    return current;
  }

  public double downShooterCurrent() {
    double current = powerDistribution.getCurrent(PowerDistributionConstants.kShooterDownMotorCurrentchannel);
    SmartDashboard.putNumber("downShooterCurrent", current);
    return current;
  }

  public double lineCurrent() {
    double current = powerDistribution.getCurrent(PowerDistributionConstants.kLineCurrentchannel);
    SmartDashboard.putNumber("lineCurrent", current);
    return current;
  }

  public double transportCurrent() {
    double current = powerDistribution.getCurrent(PowerDistributionConstants.kTransportCurrentchannel);
    SmartDashboard.putNumber("transportCurrent", current);
    return current;
  }

  public double rotateShooterCurrent() {
    double current = powerDistribution.getCurrent(PowerDistributionConstants.kRiseShooterCurrentchannel);
    SmartDashboard.putNumber("rotateShooterCurrent", current);
    return current;
  }

  public double rotateIntakeCurrent() {
    double current = powerDistribution.getCurrent(PowerDistributionConstants.kRotateIntakeCurrentchannel);
    SmartDashboard.putNumber("rotateIntakeCurrent", current);
    return current;
  }

  public boolean isIntakeOverCurrent() {
    boolean isOverCurrent = intakeCurrent() > PowerDistributionConstants.kIntakeMotorMaxCurrent;
    SmartDashboard.putBoolean("isIntakeOverCurren", isOverCurrent);
    return isOverCurrent;
  }

  public boolean isLeftOverCurrent() {
    boolean isOverCurrent = hookLeftCurrent() > PowerDistributionConstants.kHookMotor1MaxCurrent;
    SmartDashboard.putBoolean("isHookLeftOverCurrent", isOverCurrent);
    return isOverCurrent;
  }

  public boolean isRightOverCurrent() {
    boolean isOverCurrent = hookRightCurrent() > PowerDistributionConstants.kHookMotor2MaxCurrent;
    SmartDashboard.putBoolean("isHookRightOverCurrent", isOverCurrent);
    return isOverCurrent;
  }

  public boolean isShooterUpOverCurrent() {
    boolean isOverCurrent = upShooterCurrent() > PowerDistributionConstants.kShooterUpMotorMaxCurrent;
    SmartDashboard.putBoolean("isShooterUpOverCurren", isOverCurrent);
    return isOverCurrent;
  }

  public boolean isShooterDownOverCurrent() {
    boolean isOverCurrent = downShooterCurrent() > PowerDistributionConstants.kShooterDownMotorMaxCuurent;
    SmartDashboard.putBoolean("isShooterDownOverCurrent", isOverCurrent);
    return isOverCurrent;
  }

  public boolean isTransportOverCurrent() {
    boolean isOverCurrent = transportCurrent() > PowerDistributionConstants.kTransportMaxCurrent;
    SmartDashboard.putBoolean("isTransportOverCurrent", isOverCurrent);
    return isOverCurrent;

  }

  public boolean isLineMoterOverCurrent() {
    boolean isOverCurrent = lineCurrent() > PowerDistributionConstants.kLineMotorMaxCurrent;
    SmartDashboard.putBoolean("isLineMoterOverCurrent", isOverCurrent);
    return isOverCurrent;

  }

  public boolean isRotateShooterOverCurrent() {
    boolean isOverCurrent = rotateShooterCurrent() > PowerDistributionConstants.kRotateShooterMaxCurrent;
    SmartDashboard.putBoolean("isRotateShooterOverCurrent", isOverCurrent);
    return isOverCurrent;
  }

  public boolean isRotateIntakeOverCurrent() {
    boolean isOverCurrent = rotateIntakeCurrent() > PowerDistributionConstants.kRotateIntakeMaxCurrent;
    return isOverCurrent;
  }

}
