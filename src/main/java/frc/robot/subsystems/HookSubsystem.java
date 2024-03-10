// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.VictorSPXControlMode;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.HookConstants;

public class HookSubsystem extends SubsystemBase {
  /** Creates a new HookSubsystem. */
  private final PIDController linePID;
  private final PIDController leftPID;
  private final PIDController rightPID;
  private final CANSparkMax lineMotor;
  private final VictorSPX leftMotor;
  private final VictorSPX rightMotor;
  private final RelativeEncoder lineEncoder;
  private final DutyCycleEncoder leftEncoder;
  private final DutyCycleEncoder rightEncoder;
  private final PowerDistributionSubsystem powerDistributionSubsystem;
  private double leftSetpointOffset = 0.0;
  private double rightSetpointOffset = 0.0;

  public HookSubsystem(PowerDistributionSubsystem powerDistributionSubsystem) {
    lineMotor = new CANSparkMax(HookConstants.kLineChannel, MotorType.kBrushless);
    leftMotor = new VictorSPX(HookConstants.kLeftChannel);
    rightMotor = new VictorSPX(HookConstants.kRightChannel);
    linePID = new PIDController(HookConstants.kPLine, HookConstants.kILine, HookConstants.kDLine);
    leftPID = new PIDController(HookConstants.kPHook, HookConstants.kIHook, HookConstants.kDHook);
    rightPID = new PIDController(HookConstants.kPHook, HookConstants.kIHook, HookConstants.kDHook);
    lineEncoder = lineMotor.getEncoder();
    leftEncoder = new DutyCycleEncoder(HookConstants.kLeftEncoderChannel);
    rightEncoder = new DutyCycleEncoder(HookConstants.kRightEncoderChannel);
    lineEncoder.setPositionConversionFactor(HookConstants.kHookPositionConversionfactor);
    leftMotor.setInverted(HookConstants.kLeftMotorInverted);
    rightMotor.setInverted(HookConstants.kRightMotorInverted);
    lineMotor.setInverted(HookConstants.kLineMotorInverted);
    this.powerDistributionSubsystem = powerDistributionSubsystem;
  }

  private double getLineSetpoint() {
    return linePID.getSetpoint();
  }

  private double getLeftSetpoint() {
    return leftPID.getSetpoint();
  }

  private double getRightSetpoint() {
    return rightPID.getSetpoint();
  }

  private void setLineSetpoint(double setpoint) {
    final double currentSetpoint = setpoint;
    if (isExceedPhysicalLimit(currentSetpoint) != 0) {
      linePID.setSetpoint(
          isExceedPhysicalLimit(currentSetpoint) >= 1 ? HookConstants.kLinePositionMax
              : HookConstants.kLinePositionMin);
      return;
    }
    linePID.setSetpoint(currentSetpoint);
  }

  private void setLeftSetpoint(double setpoint) {
    final double currentSetpoint = setpoint + leftSetpointOffset;
    if (isExceedPhysicalLimit(currentSetpoint) != 0) {
      leftPID.setSetpoint(
          isExceedPhysicalLimit(currentSetpoint) >= 1 ? HookConstants.kHookPositionMax
              : HookConstants.kHookPositionMin);
      return;
    }
    leftPID.setSetpoint(currentSetpoint);
  }

  private void setRightSetpoint(double setpoint) {
    final double currentSetpoint = setpoint + rightSetpointOffset;
    if (isExceedPhysicalLimit(currentSetpoint) != 0) {
      rightPID.setSetpoint(
          isExceedPhysicalLimit(currentSetpoint) >= 1 ? HookConstants.kHookPositionMax
              : HookConstants.kHookPositionMin);
      return;
    }
    rightPID.setSetpoint(currentSetpoint);
  }

  private void setLeftSetpointOffset(double setpointoffset) {
    leftSetpointOffset += setpointoffset;
    if (leftSetpointOffset > 0) {
      leftSetpointOffset = 0;
    } else if (leftSetpointOffset < -HookConstants.kOffsetLimit) {
      leftSetpointOffset = -HookConstants.kOffsetLimit;
    }
  }

  private void setRightSetpointOffset(double setpointoffset) {
    rightSetpointOffset += setpointoffset;
    if (rightSetpointOffset > 0) {
      rightSetpointOffset = 0;
    } else if (rightSetpointOffset < -HookConstants.kOffsetLimit) {
      rightSetpointOffset = -HookConstants.kOffsetLimit;
    }
  }

  private void linePIDControl() {
    double lineVoltage = linePID.calculate(getLinePosition(), getLineSetpoint());
    if (Math.abs(lineVoltage) > HookConstants.kLineVoltageLimit) {
      lineVoltage = HookConstants.kLineVoltageLimit * (lineVoltage > 0 ? 1 : -1);
    }
    setLineMotorVoltage(lineVoltage);
  }

  private void leftPIDControl() {
    double leftVoltage = leftPID.calculate(getLeftPosition(), getLeftSetpoint());
    if (Math.abs(leftVoltage) > HookConstants.kLeftVoltageLimit) {
      leftVoltage = HookConstants.kLeftVoltageLimit * (leftVoltage > 0 ? 1 : -1);
    }
    setLeftMotorVoltage(leftVoltage);
  }

  private void rightPIDControl() {
    double rightVoltage = leftPID.calculate(getRightPosition(), getRightSetpoint());
    if (Math.abs(rightVoltage) > HookConstants.kRightVoltageLimit) {
      rightVoltage = HookConstants.kRightVoltageLimit * (rightVoltage > 0 ? 1 : -1);
    }
    setRightMotorVoltage(rightVoltage);
  }

  public double getLinePosition() {
    return (lineEncoder.getPosition());
  }

  public double getLeftPosition() {
    return (leftEncoder.getAbsolutePosition() * 360);
  }

  public double getRightPosition() {
    return (rightEncoder.getAbsolutePosition() * 360);
  }

  private double getLeftMotorBusVoltage() {
    return leftMotor.getBusVoltage();
  }

  private double getRightMotorBusVoltage() {
    return rightMotor.getBusVoltage();
  }

  public void stopLineMotor() {
    lineMotor.setVoltage(0.0);
  }

  public void stopLeftMotor() {
    leftMotor.set(VictorSPXControlMode.PercentOutput, 0.0);
  }

  public void stopRightMotor() {
    rightMotor.set(VictorSPXControlMode.PercentOutput, 0.0);
  }

  public void setLineMotorVoltage(double voltage) {
    if (powerDistributionSubsystem.isLineMoterOverCurrent()) {
      stopLineMotor();
      return;
    }
    lineMotor.setVoltage(voltage);
  }

  public void setLeftMotorVoltage(double voltage) {
    setLeftMotor(voltage / getLeftMotorBusVoltage());
  }

  public void setRightMotorVoltage(double voltage) {
    setRightMotor(voltage / getRightMotorBusVoltage());
  }

  private void setLeftMotor(double power) {
    if (powerDistributionSubsystem.isLeftOverCurrent()) {
      stopLeftMotor();
      return;
    }
    leftMotor.set(VictorSPXControlMode.PercentOutput, power);
  }

  private void setRightMotor(double power) {
    if (powerDistributionSubsystem.isRightOverCurrent()) {
      stopRightMotor();
      return;
    }
    rightMotor.set(VictorSPXControlMode.PercentOutput, power);
  }

  public void resetEncoder() {
    lineEncoder.setPosition(0);
    leftEncoder.reset();
    rightEncoder.reset();
  }

  private int isExceedPhysicalLimit(double position) {
    return (position < HookConstants.kLinePositionMin ? -1 : (position > HookConstants.kLinePositionMax) ? 1 : 0);
  }

  @Override
  public void periodic() {
    linePIDControl();
    leftPIDControl();
    rightPIDControl();
    // This method will be called once per scheduler run
    // SmartDashboard.putData("LINEPID", linePID);
    // SmartDashboard.putData("left hook motor", hookLeftMotorPID);
    // SmartDashboard.putData("Right hook PID", hookRightMotorPID);
  }

  // @Override
  // public void initSendable(SendableBuilder builder) {
  // builder.setSmartDashboardType("hookSubsystem");
  // builder.addDoubleProperty("lineVoltage", () -> lineMotor.get() *
  // lineMotor.getBusVoltage(), null);
  // builder.addDoubleProperty("hookLeftMotorVoltage",
  // leftMotor::getMotorOutputVoltage, null);
  // builder.addDoubleProperty("hookRightMotorVoltage",
  // rightMotor::getMotorOutputVoltage, null);
  // builder.addDoubleProperty("linePosition", lineEncoder::getPosition, null);
  // builder.addDoubleProperty("hookAbsolutePosition",
  // leftEncoder::getAbsolutePosition, null);
  // linePID.initSendable(builder);
  // leftPID.initSendable(builder);
  // rightPID.initSendable(builder);
  // }

  public Command upAllCmd() {
    Command cmd = new ParallelCommandGroup(upLineCmd(), upRightCmd(), upLeftCmd());
    cmd.setName("upAllCmd");
    return cmd;
  }

  public Command downAllCmd() {
    Command cmd = new ParallelCommandGroup(downLineCmd(), downRightCmd(), downLeftCmd());
    cmd.setName("downAllCmd");
    return cmd;
  }

  private Command upLineCmd() {
    Command cmd = Commands.run(
        () -> setLineSetpoint(getLineSetpoint() + HookConstants.kLineSetpointModify),
        this);
    cmd.setName("upLineCmd");
    return cmd;
  }

  private Command upLeftCmd() {
    Command cmd = Commands.run(
        () -> setLeftSetpoint(getLeftSetpoint() + HookConstants.kLeftSetpointModify),
        this);
    cmd.setName("upLeftCmd");
    return cmd;
  }

  private Command upRightCmd() {
    Command cmd = Commands.run(
        () -> setRightSetpoint(getRightSetpoint() + HookConstants.kRightSetpointModify),
        this);
    cmd.setName("upRightCmd");
    return cmd;
  }

  private Command downLineCmd() {
    Command cmd = Commands.run(
        () -> setLineSetpoint(getLineSetpoint() - HookConstants.kLineSetpointModify),
        this);
    cmd.setName("downKineCmd");
    return cmd;
  }

  private Command downLeftCmd() {
    Command cmd = Commands.run(
        () -> setLeftSetpoint(getLeftSetpoint() - HookConstants.kLeftSetpointModify),
        this);
    cmd.setName("downLeftCmd");
    return cmd;
  }

  private Command downRightCmd() {
    Command cmd = Commands.run(
        () -> setRightSetpoint(getRightSetpoint() - HookConstants.kRightSetpointModify),
        this);
    cmd.setName("downRightCmd");
    return cmd;
  }

  public Command leftUpIndivisual() {
    Command cmd = Commands.run(
        () -> setLeftSetpointOffset(leftSetpointOffset + HookConstants.kLeftSetpointModify),
        this);
    cmd.setName("leftUpIndivisual");
    return cmd;
  }

  public Command rightUpIndivisual() {
    Command cmd = Commands.run(
        () -> setRightSetpointOffset(rightSetpointOffset + HookConstants.kRightSetpointModify),
        this);
    cmd.setName("rightUpIndivisual");
    return cmd;
  }

  public Command leftDownIndivisual() {
    Command cmd = Commands.run(
        () -> setLeftSetpointOffset(leftSetpointOffset - HookConstants.kLeftSetpointModify),
        this);
    cmd.setName("leftUpIndivisual");
    return cmd;
  }

  public Command rightDownIndivisual() {
    Command cmd = Commands.run(
        () -> setRightSetpointOffset(rightSetpointOffset + HookConstants.kRightSetpointModify),
        this);
    cmd.setName("rightDownIndivisual");
    return cmd;
  }
}
