// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.VictorSPXControlMode;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.Constants.IntakeConstants;

public class IntakeSubsystem extends SubsystemBase {
  /** Creates a new Intake. */
  private final VictorSPX intakeMotor;
  private final PowerDistributionSubsystem powerDistributionSubsystem;

  public IntakeSubsystem(PowerDistributionSubsystem powerDistributionSubsystem) {
    this.powerDistributionSubsystem = powerDistributionSubsystem;
    intakeMotor = new VictorSPX(IntakeConstants.kIntakeChannel);
    intakeMotor.setInverted(IntakeConstants.kIntakeInverted);
  }

  public void setIntake() {
    setMotorVoltage(IntakeConstants.kIntakeVoltage);
  }

  public void setReIntake() {
    setMotorVoltage(IntakeConstants.kThrowPrecentage);
  }

  public void stopMotor() {
    setMotorVoltage(0);
  }

  private  void setMotorVoltage(double voltage) {
    if (powerDistributionSubsystem.isIntakeOverCurrent()) {
      stopMotor();
      return;
    }
    intakeMotor.set(VictorSPXControlMode.PercentOutput, voltage / getIntakeMotorBusVoltage());
  }

  private double getIntakeMotorBusVoltage() {
    return intakeMotor.getBusVoltage();
  }

  @Override
  public void periodic() {
    SmartDashboard.putNumber("IntakeVoltage", intakeMotor.getMotorOutputVoltage());
  }

  public Command intakeCmd() {
    Command cmd = Commands.runEnd(
        this::setIntake,
        this::stopMotor,
        this);
    cmd.setName("intakeCmd");
    return cmd;
  }

  public Command reIntakeCmd() {
    Command cmd = Commands.runEnd(
        this::setReIntake,
        this::stopMotor,
        this);
    cmd.setName("reIntakeCmd");
    return cmd;
  }
}
