// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.VictorSPXControlMode;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ShooterConstants;

public class ShooterSubsystem extends SubsystemBase {
  /** Creates a new Shooter. */
  private final VictorSPX upMotor;
  private final VictorSPX downMotor;
  private final Encoder upEncoder;
  private final Encoder downEncoder;
  private final PIDController ratePID;
  private final SimpleMotorFeedforward upMotorFeedForwardController;
  private final SimpleMotorFeedforward downMotorFeedForwardController;
  private final PowerDistributionSubsystem powerDistributionSubsystem;

  public ShooterSubsystem(PowerDistributionSubsystem powerDistribution) {
    upMotor = new VictorSPX(ShooterConstants.kUpMotorChannel);
    downMotor = new VictorSPX(ShooterConstants.kDownMotorChannel);
    upEncoder = new Encoder(ShooterConstants.kUpEncoderChannelA, ShooterConstants.kUpEncoderChannelB);
    downEncoder = new Encoder(ShooterConstants.kDownEncoderChannelA,
        ShooterConstants.kDownEncoderChannelB);

    upEncoder.setReverseDirection(ShooterConstants.kUpEncoderInverted);
    downEncoder.setReverseDirection(ShooterConstants.kDownEncoderInverted);

    ratePID = new PIDController(ShooterConstants.kP, ShooterConstants.kI, ShooterConstants.kD);

    upMotor.setInverted(ShooterConstants.kUpMotorInverted);
    downMotor.setInverted(ShooterConstants.kDownMotorInverted);

    upMotorFeedForwardController = new SimpleMotorFeedforward(ShooterConstants.kUpMotorS, ShooterConstants.kUpMotorV,
        ShooterConstants.kUpMotorA);
    downMotorFeedForwardController = new SimpleMotorFeedforward(ShooterConstants.kDownMotorS,
        ShooterConstants.kDownMotorV,
        ShooterConstants.kDownMotorA);

    resetEncoder();

    this.powerDistributionSubsystem = powerDistribution;
  }

  public Command speakerShootPIDCmd() {
    Command cmd = runEnd(
        this::speakerRate,
        this::stopAllMotor);
    cmd.setName("speakerShootPIDCmd");
    return cmd;
  }

  public Command ampShootPIDCmd() {
    Command cmd = runEnd(
        this::ampRate,
        this::stopAllMotor);
    cmd.setName("ampShootPIDCmd");
    return cmd;
  }

  public Command carryShootPIDCmd() {
    Command cmd = runEnd(
        this::carryRate,
        this::stopAllMotor);
    cmd.setName("carryShootPIDCmd");
    return cmd;
  }

  public void stopAllMotor() {
    stopDownMotor();
    stopUpMotor();
  }

  public void resetEncoder() {
    upEncoder.reset();
    downEncoder.reset();
  }

  public void speakerRate() {
    double upRate = ShooterConstants.kSpeakerShootRate[0];
    double downRate = ShooterConstants.kSpeakerShootRate[1];
    setRateControl(upRate, downRate);
  }

  public void ampRate() {
    double upRate = ShooterConstants.kAmpShootRate[0];
    double downRate = ShooterConstants.kAmpShootRate[1];
    setRateControl(upRate, downRate);
  }

  public void carryRate() {
    double upRate = ShooterConstants.kCarryShooterRate[0];
    double downRate = ShooterConstants.kCarryShooterRate[1];

    setRateControl(upRate, downRate);
  }

  public void setRateControl(double upRate, double downRate) {
    final double upMotorVoltage = upMotorFeedForwardController.calculate(upRate)
        + ratePID.calculate(getUpEncoderRate(), upRate);
    final double downMotorVoltage = downMotorFeedForwardController.calculate(downRate)
        + ratePID.calculate(getDownEncoderRate(), downRate);
    setUpMotorVoltage(upMotorVoltage);
    setDownMotorVoltage(downMotorVoltage);
    // SmartDashboard.putNumber("upMotorVoltage", upMotorVoltage);
    // SmartDashboard.putNumber("downMotorVoltage", downMotorVoltage);
  }

  public void stopUpMotor() {
    upMotor.set(VictorSPXControlMode.PercentOutput, 0.0);
  }

  public void stopDownMotor() {
    downMotor.set(VictorSPXControlMode.PercentOutput, 0.0);
  }

  public double getUpEncoderRate() {
    return upEncoder.getRate() / 2048.0;
  }

  public double getDownEncoderRate() {
    return downEncoder.getRate() / 2048.0;
  }

  public void setUpMotorVoltage(double voltage) {
    setUpMotor(voltage / getUpMotorBusVoltage());
  }

  public void setDownMotorVoltage(double voltage) {
    setDownMotor(voltage / getDownMotorBusVoltage());
  }

  public double getUpMotorBusVoltage() {
    return upMotor.getBusVoltage();
  }

  public double getDownMotorBusVoltage() {
    return downMotor.getBusVoltage();
  }

  public void setUpMotor(double power) {
    if (powerDistributionSubsystem.isShooterUpOverCurrent()) {
      stopUpMotor();
      return;
    }
    upMotor.set(VictorSPXControlMode.PercentOutput, power);
  }

  public void setDownMotor(double power) {
    if (powerDistributionSubsystem.isShooterDownOverCurrent()) {
      stopDownMotor();
      return;
    }
    downMotor.set(VictorSPXControlMode.PercentOutput, power);
  }

  /**
   * @param mode set shooter rate mode
   * @param 0    speaker mode
   * @param 1    amp mode
   * @param 2    carry mode
   */
  public boolean isEnoughRate(int mode) {
    switch (mode) {
      case 0:
        return (getUpEncoderRate() >= 50.0
            && getDownEncoderRate() >= 50.0);
      case 1:
        return (getUpEncoderRate() >= ShooterConstants.kAmpShootRate[0]
            && getDownEncoderRate() >= ShooterConstants.kAmpShootRate[1]);
      case 2:
        return (getUpEncoderRate() >= ShooterConstants.kCarryShooterRate[0]
            && getDownEncoderRate() >= ShooterConstants.kCarryShooterRate[1]);
      default:
        return false;
    }
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    // SmartDashboard.putNumber("upRate", getUpEncoderRate());
    // SmartDashboard.putNumber("downRate", getDownEncoderRate());
    // SmartDashboard.putNumber("upPower", upMotor.getMotorOutputPercent());
    // SmartDashboard.putNumber("downPower", downMotor.getMotorOutputPercent());
    SmartDashboard.putNumber("up rate", getUpEncoderRate());
    SmartDashboard.putNumber("down rate", getDownEncoderRate());
    SmartDashboard.putBoolean("isEnoughRate", isEnoughRate(0));
  }

  @Override
  public void initSendable(SendableBuilder builder) {
    // builder.setSmartDashboardType("ShooterSubsystem");
    // builder.addDoubleProperty("upRate", this::getUpEncoderRate, null);
    // builder.addDoubleProperty("downRate", this::getDownEncoderRate, null);
    // builder.addDoubleProperty("upVoltage", upMotor::getMotorOutputVoltage, null);
    // builder.addDoubleProperty("downVoltage", downMotor::getMotorOutputVoltage,
    // null);
    // ratePID.initSendable(builder);
  }
}
