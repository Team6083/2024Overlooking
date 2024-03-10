// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.VictorSPXControlMode;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
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
  private int rateMode=1;

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

  public void stopAllMotor() {
    stopDownMotor();
    stopUpMotor();
  }

  private void resetEncoder() {
    upEncoder.reset();
    downEncoder.reset();
  }

  private void setRateControl() {
    double upRate;
    double downRate;
    switch (rateMode) {
      case 1:
        upRate = ShooterConstants.kSpeakerShootRate[0];
        downRate = ShooterConstants.kSpeakerShootRate[1];
        break;
      case 2:
        upRate = ShooterConstants.kAmpShootRate[0];
        downRate = ShooterConstants.kAmpShootRate[1];
        break;
      case 3:
        upRate = ShooterConstants.kCarryShooterRate[0];
        downRate = ShooterConstants.kCarryShooterRate[1];
        break;
      default:
        upRate = 0;
        downRate = 0;
        break;
    }
    final double upMotorVoltage = upMotorFeedForwardController.calculate(upRate)
        + ratePID.calculate(getUpEncoderRate(), upRate);
    final double downMotorVoltage = downMotorFeedForwardController.calculate(downRate)
        + ratePID.calculate(getDownEncoderRate(), downRate);
    setUpMotorVoltage(upMotorVoltage);
    setDownMotorVoltage(downMotorVoltage);
  }

  private void stopUpMotor() {
    upMotor.set(VictorSPXControlMode.PercentOutput, 0.0);
  }

  private void stopDownMotor() {
    downMotor.set(VictorSPXControlMode.PercentOutput, 0.0);
  }

  private double getUpEncoderRate() {
    return upEncoder.getRate() / 2048.0;
  }

  private double getDownEncoderRate() {
    return downEncoder.getRate() / 2048.0;
  }

  private void setUpMotorVoltage(double voltage) {
    setUpMotor(voltage / getUpMotorBusVoltage());
  }

  private void setDownMotorVoltage(double voltage) {
    setDownMotor(voltage / getDownMotorBusVoltage());
  }

  private double getUpMotorBusVoltage() {
    return upMotor.getBusVoltage();
  }

  private double getDownMotorBusVoltage() {
    return downMotor.getBusVoltage();
  }

  private void setUpMotor(double power) {
    if (powerDistributionSubsystem.isShooterUpOverCurrent()) {
      stopUpMotor();
      return;
    }
    upMotor.set(VictorSPXControlMode.PercentOutput, power);
  }

  private void setDownMotor(double power) {
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
  public boolean isEnoughRate() {
    switch (rateMode) {
      case 1:
        return (getUpEncoderRate() >= ShooterConstants.kSpeakerShootRate[0] - ShooterConstants.kShooterRateOffset
            && getDownEncoderRate() >= ShooterConstants.kSpeakerShootRate[1] - ShooterConstants.kShooterRateOffset);
      case 2:
        return (getUpEncoderRate() >= ShooterConstants.kAmpShootRate[0] - ShooterConstants.kShooterRateOffset
            && getDownEncoderRate() >= ShooterConstants.kAmpShootRate[1] - ShooterConstants.kShooterRateOffset);
      case 3:
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
    SmartDashboard.putBoolean("isEnoughRate", isEnoughRate());
    SmartDashboard.putNumber("shooterRateMode", rateMode);
    SmartDashboard.putNumber("upMotorVoltage", upMotor.getMotorOutputVoltage());
    SmartDashboard.putNumber("downMotorVoltage", downMotor.getMotorOutputVoltage());
  }

  public Command setRateModeCmd(int mode) {
    Command cmd = runOnce(() -> setRateMode(mode));
    cmd.setName("setRateModeCmd");
    return cmd;
  }

  public Command shootPIDRateCmd() {
    Command cmd = runEnd(
        this::setRateControl,
        this::stopAllMotor);
    cmd.setName("shootPIDRateCmd");
    return cmd;
  }

  public Command resetEncoderCmd() {
    Command cmd = runOnce(
        this::resetEncoderCmd);
    cmd.setName("resetEncoderCmd");
    return cmd;
  }
}
