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
import edu.wpi.first.wpilibj2.command.Commands;
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
  private int rateMode;

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

    rateMode = 0;

    resetEncoder();

    this.powerDistributionSubsystem = powerDistribution;
  }

  public void stopAllMotor() {
    stopDownMotor();
    stopUpMotor();
  }

  public void resetEncoder() {
    upEncoder.reset();
    downEncoder.reset();
  }

  public void setSpeakerRate() {
    double upRate = ShooterConstants.kSpeakerShootRate[0];
    double downRate = ShooterConstants.kSpeakerShootRate[1];
    setRateControl(upRate, downRate);
  }

  public void setAmpRate() {
    double upRate = ShooterConstants.kAmpShootRate[0];
    double downRate = ShooterConstants.kAmpShootRate[1];
    setRateControl(upRate, downRate);
  }

  public void setCarryRate() {
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
        return (getUpEncoderRate() >= ShooterConstants.kSpeakerShootRate[0] - ShooterConstants.kShooterRateOffset
            && getDownEncoderRate() >= ShooterConstants.kSpeakerShootRate[1] - ShooterConstants.kShooterRateOffset);
      case 1:
        return (getUpEncoderRate() >= ShooterConstants.kAmpShootRate[0] - ShooterConstants.kShooterRateOffset
            && getDownEncoderRate() >= ShooterConstants.kAmpShootRate[1] - ShooterConstants.kShooterRateOffset);
      case 2:
        return (getUpEncoderRate() >= ShooterConstants.kCarryShooterRate[0] - ShooterConstants.kShooterRateOffset
            && getDownEncoderRate() >= ShooterConstants.kCarryShooterRate[1] - ShooterConstants.kShooterRateOffset);
      default:
        return false;
    }
  }

  public void setRateMode(int mode) {
    rateMode = mode;
  }

  @Override
  public void periodic() {
    SmartDashboard.putNumber("upMotorRate", getUpEncoderRate());
    SmartDashboard.putNumber("downMotorRate", getDownEncoderRate());
    SmartDashboard.putBoolean("isEnoughSpeakerRate", isEnoughRate(0));
    SmartDashboard.putBoolean("isEnoughAmpRate", isEnoughRate(1));
    SmartDashboard.putBoolean("isEnoughCarryRate", isEnoughRate(2));
  }

  public Command shootPIDRateCmd() {
    switch (rateMode) {
      case 0:
        return shootPIDRateCmd();
      case 1:
        return ampShootPIDCmd();
      case 2:
        return carryShootPIDCmd();
      default:
        return Commands.none();
    }
  }

  public Command speakerShootPIDCmd() {
    Command cmd = runEnd(
        this::setSpeakerRate,
        this::stopAllMotor);
    cmd.setName("speakerShootPIDCmd");
    return cmd;
  }

  public Command ampShootPIDCmd() {
    Command cmd = runEnd(
        this::setAmpRate,
        this::stopAllMotor);
    cmd.setName("ampShootPIDCmd");
    return cmd;
  }

  public Command carryShootPIDCmd() {
    Command cmd = runEnd(
        this::setCarryRate,
        this::stopAllMotor);
    cmd.setName("carryShootPIDCmd");
    return cmd;
  }
}
