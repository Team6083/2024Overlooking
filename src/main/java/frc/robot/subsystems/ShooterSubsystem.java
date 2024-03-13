// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.VictorSPXControlMode;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;
import com.fasterxml.jackson.databind.ser.std.StdKeySerializers.Default;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.RotateShooterConstants;
import frc.robot.Constants.ShooterConstants;
import frc.robot.subsystems.visionProcessing.TagTracking;

public class ShooterSubsystem extends SubsystemBase {
  /** Creates a new Shooter. */
  // shooter
  private final VictorSPX upShooterMotor;
  private final VictorSPX downShooterMotor;
  private final Encoder upShooterEncoder;
  private final Encoder downShooterEncoder;
  private final PIDController rateShooterPID;
  private final SimpleMotorFeedforward upMotorFeedForwardController;
  private final SimpleMotorFeedforward downMotorFeedForwardController;
  private int shootMode = 1;
  private int shooterMode = 1;
  // rotate shooter
  private final TagTracking tagTracking;
  private boolean isMaunal = false;
  private boolean isAutoAim = false;
  private double manualVoltage = 0;
  private final CANSparkMax rotateMotor;
  private final DutyCycleEncoder rotateEncoder;
  private final PIDController rotatePID;
  private double rotateDegreeError = 0.0;
  private double setPoint;
  private double upGoalRate = 0;
  private double downGoalRate = 0;

  private final PowerDistributionSubsystem powerDistributionSubsystem;

  public ShooterSubsystem(PowerDistributionSubsystem powerDistribution, TagTracking tagTracking) {
    // shooter
    upShooterMotor = new VictorSPX(ShooterConstants.kUpMotorChannel);
    downShooterMotor = new VictorSPX(ShooterConstants.kDownMotorChannel);
    upShooterEncoder = new Encoder(ShooterConstants.kUpEncoderChannelA, ShooterConstants.kUpEncoderChannelB);
    downShooterEncoder = new Encoder(ShooterConstants.kDownEncoderChannelA,
        ShooterConstants.kDownEncoderChannelB);

    upShooterEncoder.setReverseDirection(ShooterConstants.kUpEncoderInverted);
    downShooterEncoder.setReverseDirection(ShooterConstants.kDownEncoderInverted);
    rateShooterPID = new PIDController(ShooterConstants.kP, ShooterConstants.kI, ShooterConstants.kD);

    upShooterMotor.setInverted(ShooterConstants.kUpMotorInverted);
    downShooterMotor.setInverted(ShooterConstants.kDownMotorInverted);

    upMotorFeedForwardController = new SimpleMotorFeedforward(ShooterConstants.kUpMotorS, ShooterConstants.kUpMotorV,
        ShooterConstants.kUpMotorA);
    downMotorFeedForwardController = new SimpleMotorFeedforward(ShooterConstants.kDownMotorS,
        ShooterConstants.kDownMotorV,
        ShooterConstants.kDownMotorA);

    resetEncoder();

    // rotate shooter
    rotateMotor = new CANSparkMax(RotateShooterConstants.kRotateShooterChannel, MotorType.kBrushless);
    rotateMotor.setInverted(RotateShooterConstants.kRotateShooterInverted);
    rotateEncoder = new DutyCycleEncoder(RotateShooterConstants.kEncoderChannel);
    rotatePID = new PIDController(RotateShooterConstants.kP, RotateShooterConstants.kI, RotateShooterConstants.kD);
    this.powerDistributionSubsystem = powerDistribution;
    this.tagTracking = tagTracking;
    rotatePID.enableContinuousInput(-180.0, 180.0);
    setSetpoint(RotateShooterConstants.kInitDegree);
  }

  public double getSetpoint() {
    return rotatePID.getSetpoint();
  }

  public void setSetpoint(double setpoint) {
    final double currentSetpoint = getSetpoint() + rotateDegreeError;
    if (hasExceedPhysicalLimit(currentSetpoint) != 0) {

      return;
    }
    if (hasExceedPhysicalLimit(setpoint) == -1) {
      setpoint = RotateShooterConstants.kRotateAngleMin;
    } else if (hasExceedPhysicalLimit(setpoint) == 1) {
      setpoint = RotateShooterConstants.kRotateAngleMax;
    }
    rotatePID.setSetpoint(setpoint);
  }

  private void setPIDControl() {
    double rotateVoltage = rotatePID.calculate(getAngle());
    double modifiedRotateVoltage = rotateVoltage;
    if (Math.abs(modifiedRotateVoltage) > RotateShooterConstants.kRotateVoltLimit) {
      modifiedRotateVoltage = RotateShooterConstants.kRotateVoltLimit * (rotateVoltage > 0 ? 1 : -1);
    }
    setMotor(modifiedRotateVoltage);
  }

  public double getAngle() {
    double degree = (RotateShooterConstants.kEncoderInverted ? -1.0 : 1.0)
        * ((rotateEncoder.getAbsolutePosition() * 360.0) - RotateShooterConstants.kRotateOffset
            - RotateShooterConstants.kShooterOffset)
        % 360.0;
    if (Math.abs(degree) > 180) {
      degree -= 360;
    }
    return degree;
  }

  public double getSpeakerDegree(double currentDegree) {
    if (tagTracking.getTv() == 1 && tagTracking.getTID() != 3.0
        && tagTracking.getTID() != 8.0 && tagTracking.getHorDistanceByCal() > 1.1) {
      double horizonDistance = tagTracking.getHorizontalDistanceByCT();
      double speakerToShooterHeight = RotateShooterConstants.kSpeakerHeight - RotateShooterConstants.kShooterHeight;
      double degree = Math.toDegrees(Math.atan(speakerToShooterHeight / horizonDistance));
      return degree + 9.0 * horizonDistance / 2.9;
    }
    return currentDegree;
  }

  public double getAmpDegree(double currentDegree) {
    if (tagTracking.getTv() == 1) {
      double ampToShooterHeight = RotateShooterConstants.kAMPHeight - RotateShooterConstants.kShooterHeight;
      double degree = Math.toDegrees(Math.atan(ampToShooterHeight / tagTracking.getHorizontalDistanceByCT()));
      return degree;
    }
    return currentDegree;
  }

  public void autoAimOn() {
    this.isAutoAim = true;
  }

  public void autoAimOff() {
    this.isAutoAim = false;
  }

  public Command setAutoAimCmd() {
    Command cmd = runEnd(() -> autoAimOn(), () -> autoAimOff()).onlyWhile(() -> shootMode != 2);
    cmd.setName("setAutoAimCmd");
    return cmd;
  }

  private void setManualVoltage(double voltage) {
    manualVoltage = voltage;
  }

  public Command setManualVoltageCmd(double voltage) {
    Command cmd = runOnce(() -> setManualVoltage(voltage));
    cmd.setName("SetManualVoltageCmd");
    return cmd;
  }

  /**
   * Stop both up and down shooter motor.
   */
  public void stopAllMotor() {
    stopDownMotor();
    stopUpMotor();
  }

  /**
   * Reset both up and down encoders.
   */
  private void resetEncoder() {
    upShooterEncoder.reset();
    downShooterEncoder.reset();
  }

  private void setSpeakerRateControl() {
  setSetpoint(getSpeakerDegree(getSetpoint()));
  double upRate = ShooterConstants.kSpeakerShooterRate[0];
  double downRate = ShooterConstants.kSpeakerShooterRate[1];
  final double upMotorVoltage = upMotorFeedForwardController.calculate(upRate)
  + rateShooterPID.calculate(getUpEncoderRate(), upRate);
  final double downMotorVoltage =
  downMotorFeedForwardController.calculate(downRate)
  + rateShooterPID.calculate(getDownEncoderRate(), downRate);
  setUpMotorVoltage(upMotorVoltage);
  setDownMotorVoltage(downMotorVoltage);
  }

  // private void setCarryRateControl() {
  // setSetpoint(RotateShooterConstants.kCarryDegree);
  // double upRate = ShooterConstants.kCarryShooterRate[0];
  // double downRate = ShooterConstants.kCarryShooterRate[1];
  // final double upMotorVoltage = upMotorFeedForwardController.calculate(upRate)
  // + rateShooterPID.calculate(getUpEncoderRate(), upRate);
  // final double downMotorVoltage =
  // downMotorFeedForwardController.calculate(downRate)
  // + rateShooterPID.calculate(getDownEncoderRate(), downRate);
  // setUpMotorVoltage(upMotorVoltage);
  // setDownMotorVoltage(downMotorVoltage);
  // }

  // private void setInitRateControl() {
  // setSetpoint(RotateShooterConstants.kInitDegree);
  // double upRate = ShooterConstants.kInitShooterRate[0];
  // double downRate = ShooterConstants.kInitShooterRate[1];
  // final double upMotorVoltage = upMotorFeedForwardController.calculate(upRate)
  // + rateShooterPID.calculate(getUpEncoderRate(), upRate);
  // final double downMotorVoltage =
  // downMotorFeedForwardController.calculate(downRate)
  // + rateShooterPID.calculate(getDownEncoderRate(), downRate);
  // setUpMotorVoltage(upMotorVoltage);
  // setDownMotorVoltage(downMotorVoltage);
  // }

  public void setAdjustAngleByTag() {
    setPoint = getSpeakerDegree(getAngle());
  }

  public void setFixAngle() {
    setPoint = RotateShooterConstants.kInitDegree;
  }

  // write these two methods into cmd

  public Command setAdjustAngleByTagCommand() {
    Command cmd = runOnce(this::setAdjustAngleByTag);
    cmd.setName("setAdjustAngleByTag");
    return cmd;
  }

  public Command setFixAngleCommand() {
    Command cmd = runOnce(this::setFixAngle);
    cmd.setName("setFixAngle");
    return cmd;
  }

  public void shootRateControlMode() {
    setSetpoint(setPoint);
  }

  public void shootRateControl() {
    // shootRateControlMode();
    upGoalRate = ShooterConstants.kInitShooterRate[0];
    downGoalRate = ShooterConstants.kInitShooterRate[1];
    final double upMotorVoltage = upMotorFeedForwardController.calculate(upGoalRate)
        + rateShooterPID.calculate(getUpEncoderRate(), upGoalRate);
    final double downMotorVoltage = downMotorFeedForwardController.calculate(downGoalRate)
        + rateShooterPID.calculate(getDownEncoderRate(), downGoalRate);
    setUpMotorVoltage(upMotorVoltage);
    setDownMotorVoltage(downMotorVoltage);
  }

  public Command shootRateControlModeCmd() {
    Command cmd = runOnce(this::shootRateControlMode);
    return cmd;
  }

  public Command shootRateControlCmd() {
    Command cmd = runOnce(this::shootRateControl);
    return cmd;
  }

  private void setCarryRateControl() {
    setSetpoint(RotateShooterConstants.kCarryDegree);
    upGoalRate = ShooterConstants.kCarryShooterRate[0];
    downGoalRate = ShooterConstants.kCarryShooterRate[1];
    final double upMotorVoltage = upMotorFeedForwardController.calculate(upGoalRate)
        + rateShooterPID.calculate(getUpEncoderRate(), upGoalRate);
    final double downMotorVoltage = downMotorFeedForwardController.calculate(downGoalRate)
        + rateShooterPID.calculate(getDownEncoderRate(), downGoalRate);
    setUpMotorVoltage(upMotorVoltage);
    setDownMotorVoltage(downMotorVoltage);
  }

  public Command setCarryRateControlCmd() {
    Command cmd = runOnce(this::setCarryRateControl);
    return cmd;
  }

  private void setInitRateControlMode() {
    setSetpoint(RotateShooterConstants.kInitDegree);
  }

  private void setInitRateControl() {
    upGoalRate = ShooterConstants.kInitShooterRate[0];
    downGoalRate = ShooterConstants.kInitShooterRate[1];
    final double upMotorVoltage = upMotorFeedForwardController.calculate(upGoalRate)
        + rateShooterPID.calculate(getUpEncoderRate(), upGoalRate);
    final double downMotorVoltage = downMotorFeedForwardController.calculate(downGoalRate)
        + rateShooterPID.calculate(getDownEncoderRate(), downGoalRate);
    setUpMotorVoltage(upMotorVoltage);
    setDownMotorVoltage(downMotorVoltage);
  }

  public Command setInitRateControlCmd() {
    Command cmd = runEnd(this::setInitRateControlMode, this::setInitRateControl);
    return cmd;
  }

  // public Command setInitRateControlCmd() {
  // Command cmd = runOnce(this::setInitRateControl);
  // return cmd;
  // }
  //

  // public void addManualAngleOffset() {
  //   double offset = 0;
  //   offset += 3;
  // }

  // public Command addManualAngleOffsetCmd() {
  //   Command cmd = runOnce(this::addManualAngleOffset);
  //   setName("setManualAngleOffsetCmd");
  //   return cmd;
  // }

  // public void minusManualAngleOffset() {
  //   double offset = 0;
  //   offset += 3;
  // }

  // public Command minusManualAngleOffsetCmd() {
  //   Command cmd = runOnce(this::addManualAngleOffset);
  //   setName("minusManualAngleOffsetCmd");
  //   return cmd;
  // }

  public void AdjustShooterAngleManual(double adjust){
    double offset = getAngle();
    // setSetpoint(offset+adjust);
    double rotateVoltage = rotatePID.calculate(offset+adjust);
    double modifiedRotateVoltage = rotateVoltage;
    if (Math.abs(modifiedRotateVoltage) > RotateShooterConstants.kRotateVoltLimit) {
      modifiedRotateVoltage = RotateShooterConstants.kRotateVoltLimit * (rotateVoltage > 0 ? 1 : -1);
    }
    setMotor(modifiedRotateVoltage);
  }

  /**
   * Stop up motor.
   */
  private void stopUpMotor() {
    upShooterMotor.set(VictorSPXControlMode.PercentOutput, 0.0);
  }

  /**
   * Stop down motor.
   */
  private void stopDownMotor() {
    downShooterMotor.set(VictorSPXControlMode.PercentOutput, 0.0);
  }

  /**
   * Get up encoder rate.
   * 
   * @return rate/2048 (double)
   */
  private double getUpEncoderRate() {
    return upShooterEncoder.getRate() / 2048.0;
  }

  /**
   * Get down encoder rate.
   * 
   * @return rate/2048 (double)
   */
  private double getDownEncoderRate() {
    return downShooterEncoder.getRate() / 2048.0;
  }

  /**
   * Set up motor voltage to a specific rate, so the speed won't be affected by
   * the busvoltage now.
   * 
   * @param voltage
   */
  private void setUpMotorVoltage(double voltage) {
    setUpMotor(voltage / getUpMotorBusVoltage());
  }

  /**
   * Set down motor voltage to a specific rate, so the speed won't be affected by
   * the busvoltage now.
   * 
   * @param voltage
   */
  private void setDownMotorVoltage(double voltage) {
    setDownMotor(voltage / getDownMotorBusVoltage());
  }

  /**
   * Get up motor bus voltage.
   * 
   * @return voltage
   */
  private double getUpMotorBusVoltage() {
    return upShooterMotor.getBusVoltage();
  }

  /**
   * Get down motor bus voltage.
   * 
   * @return voltage
   */
  private double getDownMotorBusVoltage() {
    return downShooterMotor.getBusVoltage();
  }

  /**
   * Set up motor voltage, stop the motor if maximum/minimum power exceeded.
   * 
   * @param power
   */
  private void setUpMotor(double power) {
    if (powerDistributionSubsystem.isShooterUpOverCurrent()) {
      stopUpMotor();
      return;
    }
    upShooterMotor.set(VictorSPXControlMode.PercentOutput, power);
  }

  /**
   * Set up motor voltage, stop the motor if maximum/minimum power exceeded.
   * 
   * @param power
   */
  private void setDownMotor(double power) {
    if (powerDistributionSubsystem.isShooterDownOverCurrent()) {
      stopDownMotor();
      return;
    }
    downShooterMotor.set(VictorSPXControlMode.PercentOutput, power);
  }

  /**
   * @param mode set shooter rate mode
   * @param 1    speaker mode
   * @param 2    amp mode
   * @param 3    carry mode
   */
  public boolean isEnoughRate() {
    switch (shootMode) {
      case 1:
        return (getUpEncoderRate() >= ShooterConstants.kSpeakerShooterRate[0] - ShooterConstants.kShooterRateOffset
            && getDownEncoderRate() >= ShooterConstants.kSpeakerShooterRate[1] - ShooterConstants.kShooterRateOffset);
      case 2:
        return (getUpEncoderRate() >= ShooterConstants.kCarryShooterRate[0] - ShooterConstants.kShooterRateOffset
            && getDownEncoderRate() >= ShooterConstants.kCarryShooterRate[1] - ShooterConstants.kShooterRateOffset);
      case 3:
        return (getUpEncoderRate() >= ShooterConstants.kInitShooterRate[0] - ShooterConstants.kShooterRateOffset
            && getDownEncoderRate() >= ShooterConstants.kInitShooterRate[1] - ShooterConstants.kShooterRateOffset);
      default:
        return false;
    }
  }

  public boolean checkIfEnoughRate() {
    return getUpEncoderRate() >= upGoalRate - ShooterConstants.kShooterRateOffset
        && getDownEncoderRate() >= downGoalRate - ShooterConstants.kShooterRateOffset;
  }

  /**
   * Set shooter rate mode.
   * 
   * @param mode 1, 2, 3
   */
  public void setShootMode(int mode) {
    shootMode = mode;
  }

  @Override
  public void periodic() {
    setPIDControl();
    SmartDashboard.putNumber("upMotorRate", getUpEncoderRate());
    SmartDashboard.putNumber("downMotorRate", getDownEncoderRate());
    SmartDashboard.putBoolean("isEnoughRate", isEnoughRate());
    SmartDashboard.putNumber("shooterRateMode", shootMode);
    SmartDashboard.putNumber("upMotorVoltage", upShooterMotor.getMotorOutputVoltage());
    SmartDashboard.putNumber("downMotorVoltage", downShooterMotor.getMotorOutputVoltage());
  }

  public void stopMotor() {
    rotateMotor.setVoltage(0.0);
  }

  public void setMotor(double voltage) {
    if (powerDistributionSubsystem.isRotateShooterOverCurrent()) {
      stopMotor();
      return;
    }
    rotateMotor.setVoltage(voltage);
  }

  private int hasExceedPhysicalLimit(double angle) {
    return (angle < RotateShooterConstants.kRotateAngleMin ? -1
        : (angle > RotateShooterConstants.kRotateAngleMax ? 1 : 0));
  }

  public void changeMaunalMode(boolean isManual) {
    this.isMaunal = isManual;
  }

  public Command speakerRateControlCmd() {
  Command cmd = runEnd(this::setSpeakerRateControl, this::stopAllMotor);
  cmd.setName("setSpeakerRateControlCmd");
  return cmd;
  }

  // public Command setCarryRateControlCmd() {
  // Command cmd = runEnd(this::setCarryRateControl, this::stopAllMotor);
  // cmd.setName("setCarryRateControlCmd");
  // return cmd;
  // }

  // public Command setInitRateControlCmd() {
  // Command cmd = runEnd(this::setInitRateControl, this::stopAllMotor);
  // cmd.setName("setInitRateControlCmd");
  // return cmd;
  // }

  /**
   * Reverse current manual mode.
   * 
   * @param mode
   * @return changeManualModeCmd
   */
  public Command changeMaunalModeCmd(boolean mode) {
    Command cmd = runOnce(() -> changeMaunalMode(mode));
    cmd.setName("changeMaunalModeCmd");
    return cmd;
  }

  /**
   * Command of resetting encoder.
   * 
   * @return resetEncoderCmd
   */
  public Command resetEncoderCmd() {
    Command cmd = runOnce(
        this::resetEncoderCmd);
    cmd.setName("resetEncoderCmd");
    return cmd;
  }

  public Command stopAllMotorCmd() {
    Command cmd = runOnce(this::stopAllMotor);
    cmd.setName("stopAllMotor");
    return cmd;
  }
}
