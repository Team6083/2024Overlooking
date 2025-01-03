// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.VictorSPXControlMode;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;

import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.IntakeConstants;

public class IntakeSubsystem extends SubsystemBase {
  /** Creates a new Intake. */
  private final VictorSPX intakeMotor;
  private final VictorSPX rotateIntake;
  private final DutyCycleEncoder rotateEncoder;
  private final PowerDistributionSubsystem powerDistributionSubsystem;

  public IntakeSubsystem(PowerDistributionSubsystem powerDistributionSubsystem) {
    this.powerDistributionSubsystem = powerDistributionSubsystem;
    intakeMotor = new VictorSPX(IntakeConstants.kIntakeChannel);
    intakeMotor.setInverted(IntakeConstants.kIntakeInverted);
    rotateIntake = new VictorSPX(IntakeConstants.kRotateIntakeChannel);
    rotateIntake.setInverted(IntakeConstants.kRotateIntakeInverted);
    rotateEncoder = new DutyCycleEncoder(IntakeConstants.kRotateEncoderChannel);
    rotateEncoder.reset();
  }

  private void setIntake() {
    setIntakeMotorVoltage(IntakeConstants.kIntakeVoltage);
  }

  private void setReIntake() {
    setIntakeMotorVoltage(IntakeConstants.kThrowVoltage);
  }

  private void setDownIntake() {
    setRotateMotorVoltage(-IntakeConstants.kRotateVoltage);
  }

  private void setUpIntake(){
    setRotateMotorVoltage(IntakeConstants.kRotateVoltage);
  }

  public Command setDownIntakeCmd(){
    Command cmd = runEnd(this::setDownIntake, this::stopRotateIntakeMotor);
    return cmd;
  }

  public Command setUpIntakeCmd(){
    Command cmd = runEnd(this::setUpIntake, this::stopRotateIntakeMotor);
    return cmd;
  }

  private void stopIntakeMotor() {
    intakeMotor.set(VictorSPXControlMode.PercentOutput, 0.0);
  }

  public void stopRotateIntakeMotor(){
    rotateIntake.set(VictorSPXControlMode.PercentOutput, 0.0);
  }

  private void setIntakeMotorVoltage(double voltage) {
    if (powerDistributionSubsystem.isIntakeOverCurrent()) {
      stopIntakeMotor();
      return;
    }
    intakeMotor.set(VictorSPXControlMode.PercentOutput, voltage / getIntakeMotorBusVoltage());
  }

  public void setRotateMotorVoltage(double voltage) {
    if (powerDistributionSubsystem.isRotateIntakeOverCurrent()) {
      stopIntakeMotor();
      return;
    }
    rotateIntake.set(VictorSPXControlMode.PercentOutput, voltage / getRotateIntakeMotorBusVoltage());
  }

  private double getIntakeMotorBusVoltage() {
    return intakeMotor.getBusVoltage();
  }

  private double getRotateIntakeMotorBusVoltage() {
    return rotateIntake.getBusVoltage();
  }

  private double getAngle() {
    double degree = (IntakeConstants.kRotateEncoderInverted ? -1.0 : 1.0)
        * ((rotateEncoder.getAbsolutePosition() * 360.0) - IntakeConstants.kRotateOffset);
    return degree;
  }

  @Override
  public void periodic() {
    SmartDashboard.putNumber("rotateIntakeVoltage", rotateIntake.getMotorOutputVoltage());
    SmartDashboard.putNumber("intakeVoltage", intakeMotor.getMotorOutputVoltage());
    SmartDashboard.putNumber("intakeDegree", getAngle());
  }

  public Command intakeCmd() {
    Command cmd = Commands.runEnd(
        this::setIntake,
        this::stopIntakeMotor,
        this);
    cmd.setName("intakeCmd");
    return cmd;
  }

  public Command reIntakeCmd() {
    Command cmd = Commands.runEnd(
        this::setReIntake,
        this::stopIntakeMotor,
        this);
    cmd.setName("reIntakeCmd");
    return cmd;
  }
}
