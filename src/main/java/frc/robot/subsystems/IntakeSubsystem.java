// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.VictorSPXControlMode;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;

import edu.wpi.first.util.sendable.SendableBuilder;
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

  public Command setIntakingCmd() {
    Command setIntaking = Commands.startEnd(() -> setIntaking(), () -> stopMotor());
    setIntaking.setName("setIntaking");
    return setIntaking;
  }

  public Command setReIntakingCmd(){
  Command setReIntaking = Commands.startEnd(() -> setThrowing(), () -> stopMotor());
  setReIntaking.setName("setRetaking");
  return setReIntaking;
  }

  public void setIntaking() {
    setMotor(IntakeConstants.kIntakeVoltage);
  }

  public void setThrowing() {
    setMotor(IntakeConstants.kThrowPrecentage);
  }

  public void stopMotor() {
    setMotor(0);
  }

  public void setMotor(double voltage) {
    if (powerDistributionSubsystem.isIntakeOverCurrent()) {
      stopMotor();
      return;
    }
    intakeMotor.set(VictorSPXControlMode.PercentOutput, voltage / getIntakeMotorBusVoltage());
  }

  public double getIntakeMotorBusVoltage() {
    return intakeMotor.getBusVoltage();
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    // SmartDashboard.getNumber("IntakeMotorBusVoltage", getIntakeMotorBusVoltage());

  }

  @Override
  public void initSendable(SendableBuilder builder){
    builder.setSmartDashboardType("IntakeSubsystem");
    builder.addDoubleProperty("IntakeMotorBusVoltage",() -> getIntakeMotorBusVoltage(),null);
  }
}
