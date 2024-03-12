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
  private final VictorSPX upMotor;
  private final VictorSPX downMotor;
  private final Encoder upEncoder;
  private final Encoder downEncoder;
  private final PIDController ratePID;
  private final CANSparkMax rotateMotor;
  private final DutyCycleEncoder rotateEncoder;
  private final PIDController rotatePID;
  private final SimpleMotorFeedforward upMotorFeedForwardController;
  private final SimpleMotorFeedforward downMotorFeedForwardController;
  private final PowerDistributionSubsystem powerDistributionSubsystem;
  private final TagTracking tagTracking;
  private int shootMode = 1;
  private boolean isMaunal = false;
  private boolean isAutoAim = false;
  private double manualVoltage = 0;
  private double rotateDegreeError = 0.0;

  public ShooterSubsystem(PowerDistributionSubsystem powerDistribution, TagTracking tagTracking) {
    upMotor = new VictorSPX(ShooterConstants.kUpMotorChannel);
    downMotor = new VictorSPX(ShooterConstants.kDownMotorChannel);
    upEncoder = new Encoder(ShooterConstants.kUpEncoderChannelA, ShooterConstants.kUpEncoderChannelB);
    downEncoder = new Encoder(ShooterConstants.kDownEncoderChannelA,
        ShooterConstants.kDownEncoderChannelB);

    upEncoder.setReverseDirection(ShooterConstants.kUpEncoderInverted);
    downEncoder.setReverseDirection(ShooterConstants.kDownEncoderInverted);

    ratePID = new PIDController(ShooterConstants.kP, ShooterConstants.kI, ShooterConstants.kD);
    rotatePID = new PIDController(RotateShooterConstants.kP, RotateShooterConstants.kI, RotateShooterConstants.kD);

    upMotor.setInverted(ShooterConstants.kUpMotorInverted);
    downMotor.setInverted(ShooterConstants.kDownMotorInverted);

    upMotorFeedForwardController = new SimpleMotorFeedforward(ShooterConstants.kUpMotorS, ShooterConstants.kUpMotorV,
        ShooterConstants.kUpMotorA);
    downMotorFeedForwardController = new SimpleMotorFeedforward(ShooterConstants.kDownMotorS,
        ShooterConstants.kDownMotorV,
        ShooterConstants.kDownMotorA);

    resetEncoder();
    rotateMotor = new CANSparkMax(RotateShooterConstants.kRotateShooterChannel, MotorType.kBrushless);
    rotateMotor.setInverted(RotateShooterConstants.kRotateShooterInverted);
    rotateEncoder = new DutyCycleEncoder(RotateShooterConstants.kEncoderChannel);
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
    upEncoder.reset();
    downEncoder.reset();
  }

  // /**
  //  * @param 1 aim degree
  //  * @param 2 carry degree
  //  * @param 3 init degree
  //  */
  // public void setModeSetpoint() {
  //   switch (shootMode) {
  //     case 1:
  //       setSetpoint(getSpeakerDegree(getSetpoint()));
  //       break;
  //     case 2:
  //       setSetpoint(RotateShooterConstants.kCarryDegree);
  //       break;
  //     case 3:
  //       setSetpoint(RotateShooterConstants.kInitDegree);
  //     default:
  //       break;
  //   }
  // }

  // /**
  //  * Set up and down voltage by using both feedforward controller and
  //  * pidcontroller to calculate the rate.
  //  */
  // private void setRateControl() {
  //   double upRate;
  //   double downRate;
  //   switch (shootMode) {
  //     case 1:
  //       upRate = ShooterConstants.kSpeakerShooterRate[0];
  //       downRate = ShooterConstants.kSpeakerShooterRate[1];
  //       break;
  //     case 2:
  //       upRate = ShooterConstants.kCarryShooterRate[0];
  //       downRate = ShooterConstants.kCarryShooterRate[1];
  //       break;
  //     default:
  //       upRate = ShooterConstants.kInitShooterRate[0];
  //       downRate = ShooterConstants.kInitShooterRate[1];
  //       break;
  //   }
  //   final double upMotorVoltage = upMotorFeedForwardController.calculate(upRate)
  //       + ratePID.calculate(getUpEncoderRate(), upRate);
  //   final double downMotorVoltage = downMotorFeedForwardController.calculate(downRate)
  //       + ratePID.calculate(getDownEncoderRate(), downRate);
  //   setUpMotorVoltage(upMotorVoltage);
  //   setDownMotorVoltage(downMotorVoltage);
  // }

  private void setSpeakerRateControl() {
    setSetpoint(getSpeakerDegree(getSetpoint()));
    double upRate = ShooterConstants.kSpeakerShooterRate[0];
    double downRate = ShooterConstants.kSpeakerShooterRate[1];
    final double upMotorVoltage = upMotorFeedForwardController.calculate(upRate)
        + ratePID.calculate(getUpEncoderRate(), upRate);
    final double downMotorVoltage = downMotorFeedForwardController.calculate(downRate)
        + ratePID.calculate(getDownEncoderRate(), downRate);
    setUpMotorVoltage(upMotorVoltage);
    setDownMotorVoltage(downMotorVoltage);
  }

  private void setCarryRateControl() {
    setSetpoint(RotateShooterConstants.kCarryDegree);
    double upRate = ShooterConstants.kCarryShooterRate[0];
    double downRate = ShooterConstants.kCarryShooterRate[1];
    final double upMotorVoltage = upMotorFeedForwardController.calculate(upRate)
        + ratePID.calculate(getUpEncoderRate(), upRate);
    final double downMotorVoltage = downMotorFeedForwardController.calculate(downRate)
        + ratePID.calculate(getDownEncoderRate(), downRate);
    setUpMotorVoltage(upMotorVoltage);
    setDownMotorVoltage(downMotorVoltage);
  }

  private void setInitRateControl() {
          setSetpoint(RotateShooterConstants.kInitDegree);
    double upRate = ShooterConstants.kInitShooterRate[0];
    double downRate = ShooterConstants.kInitShooterRate[1];
    final double upMotorVoltage = upMotorFeedForwardController.calculate(upRate)
        + ratePID.calculate(getUpEncoderRate(), upRate);
    final double downMotorVoltage = downMotorFeedForwardController.calculate(downRate)
        + ratePID.calculate(getDownEncoderRate(), downRate);
    setUpMotorVoltage(upMotorVoltage);
    setDownMotorVoltage(downMotorVoltage);
  }

  /**
   * Stop up motor.
   */
  private void stopUpMotor() {
    upMotor.set(VictorSPXControlMode.PercentOutput, 0.0);
  }

  /**
   * Stop down motor.
   */
  private void stopDownMotor() {
    downMotor.set(VictorSPXControlMode.PercentOutput, 0.0);
  }

  /**
   * Get up encoder rate.
   * 
   * @return rate/2048 (double)
   */
  private double getUpEncoderRate() {
    return upEncoder.getRate() / 2048.0;
  }

  /**
   * Get down encoder rate.
   * 
   * @return rate/2048 (double)
   */
  private double getDownEncoderRate() {
    return downEncoder.getRate() / 2048.0;
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
    return upMotor.getBusVoltage();
  }

  /**
   * Get down motor bus voltage.
   * 
   * @return voltage
   */
  private double getDownMotorBusVoltage() {
    return downMotor.getBusVoltage();
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
    upMotor.set(VictorSPXControlMode.PercentOutput, power);
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
    downMotor.set(VictorSPXControlMode.PercentOutput, power);
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
    SmartDashboard.putNumber("upMotorVoltage", upMotor.getMotorOutputVoltage());
    SmartDashboard.putNumber("downMotorVoltage", downMotor.getMotorOutputVoltage());
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

  // /**
  //  * Command of setting rate mode.
  //  * 
  //  * @param mode 1, 2, 3
  //  * @return setRateModeCmd
  //  */
  // public Command setShootModeCmd(int mode) {
  //   Command cmd = runOnce(() -> setShootMode(mode));
  //   cmd.setName("SetShootModeCmd");
  //   return cmd;
  // }

  /**
   * Command of setting shooter voltage by PID and feedforward.
   * 
   * @return shootRateControlCmd
   */
  // public Command shootPIDRateCmd() {
  // Command cmd = runEnd(
  // this::setRateControl,
  // this::stopAllMotor);
  // cmd.setName("shootPIDRateCmd");
  // return cmd;
  // }

  public Command setSpeakerRateControlCmd() {
    Command cmd = runEnd(this::setSpeakerRateControl, this::stopAllMotor);
    cmd.setName("setSpeakerRateControlCmd");
    return cmd;
  }

  public Command setCarryRateControlCmd() {
    Command cmd = runEnd(this::setCarryRateControl, this::stopAllMotor);
    cmd.setName("setCarryRateControlCmd");
    return cmd;
  }

  public Command setInitRateControlCmd() {
    Command cmd = runEnd(this::setInitRateControl, this::stopAllMotor);
    cmd.setName("setInitRateControlCmd");
    return cmd;
  }

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
}
