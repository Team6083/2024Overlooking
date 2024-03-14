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
  private final RelativeEncoder relaRotateEncoder;
  private final PIDController rotatePID;
  private double rotateDegreeError = 0.0;
  private double setPoint = RotateShooterConstants.kInitDegree;
  private double upGoalRate = 0;
  private double downGoalRate = 0;
  private double offset = 0;

  public ShooterSubsystem(TagTracking tagTracking) {
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
    this.tagTracking = tagTracking;
    relaRotateEncoder = rotateMotor.getEncoder();
    relaRotateEncoder.setPositionConversionFactor(360.0);
    relaRotateEncoder.setPosition(getAngle());
    rotatePID.enableContinuousInput(-180.0, 180.0);
    rotatePID.setSetpoint(57);
    ;
  }

  public void stopMotor() {
    rotateMotor.setVoltage(0.0);
  }

  public void setMotor(double voltage) {
    rotateMotor.setVoltage(voltage);
  }

  private int hasExceedPhysicalLimit(double angle) {
    return (angle < RotateShooterConstants.kRotateAngleMin ? -1
        : (angle > RotateShooterConstants.kRotateAngleMax ? 1 : 0));
  }

  public double getSetpoint() {
    return rotatePID.getSetpoint();
  }

  public void setSetpoint(double setpoint) {
    double rotateDegree = setpoint + rotateDegreeError;
    if (hasExceedPhysicalLimit(rotateDegree) == -1) {
      rotateDegree = RotateShooterConstants.kRotateAngleMin;
    } else if (hasExceedPhysicalLimit(rotateDegree) == 1) {
      rotateDegree = RotateShooterConstants.kRotateAngleMax;
    }
    rotatePID.setSetpoint(rotateDegree);
  }

  private void setPIDControl() {
    double rotateVoltage = rotatePID.calculate(relaRotateEncoder.getPosition());
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

  private void setManualVoltage(double voltage) {
    manualVoltage = voltage;
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

  // private void setSpeakerRateControl() {
  //   setSetpoint(getSpeakerDegree(getSetpoint()));
  //   double upRate = ShooterConstants.kSpeakerShooterRate[0];
  //   double downRate = ShooterConstants.kSpeakerShooterRate[1];
  //   final double upMotorVoltage = upMotorFeedForwardController.calculate(upRate)
  //       + rateShooterPID.calculate(getUpEncoderRate(), upRate);
  //   final double downMotorVoltage = downMotorFeedForwardController.calculate(downRate)
  //       + rateShooterPID.calculate(getDownEncoderRate(), downRate);
  //   setUpMotorVoltage(upMotorVoltage);
  //   setDownMotorVoltage(downMotorVoltage);
  // }

  public void aimControl() {
    setSetpoint(getSpeakerDegree(getSetpoint()));
    upGoalRate = ShooterConstants.kSpeakerShooterRate[0];
    downGoalRate = ShooterConstants.kSpeakerShooterRate[1];
    final double upMotorVoltage = upMotorFeedForwardController.calculate(upGoalRate)
        + rateShooterPID.calculate(getUpEncoderRate(), upGoalRate);
    final double downMotorVoltage = downMotorFeedForwardController.calculate(downGoalRate)
        + rateShooterPID.calculate(getDownEncoderRate(), downGoalRate);
    setUpMotorVoltage(upMotorVoltage);
    setDownMotorVoltage(downMotorVoltage);
    shootMode = 1;
  }

  public void transportMode() {
    setSetpoint(15);
    upGoalRate = ShooterConstants.kCarryShooterRate[0];
    downGoalRate = ShooterConstants.kCarryShooterRate[1];
    final double upMotorVoltage = upMotorFeedForwardController.calculate(upGoalRate)
        + rateShooterPID.calculate(getUpEncoderRate(), upGoalRate);
    final double downMotorVoltage = downMotorFeedForwardController.calculate(downGoalRate)
        + rateShooterPID.calculate(getDownEncoderRate(), downGoalRate);
    setUpMotorVoltage(upMotorVoltage);
    setDownMotorVoltage(downMotorVoltage);
    shootMode = 2;
  }

  private void setInitRateControl() {
    setSetpoint(RotateShooterConstants.kInitDegree);
    upGoalRate = ShooterConstants.kInitShooterRate[0];
    downGoalRate = ShooterConstants.kInitShooterRate[1];
    final double upMotorVoltage = upMotorFeedForwardController.calculate(upGoalRate)
        + rateShooterPID.calculate(getUpEncoderRate(), upGoalRate);
    final double downMotorVoltage = downMotorFeedForwardController.calculate(downGoalRate)
        + rateShooterPID.calculate(getDownEncoderRate(), downGoalRate);
    setUpMotorVoltage(upMotorVoltage);
    setDownMotorVoltage(downMotorVoltage);
    shootMode = 3;
  }

    private void setFixRateControl() {
    setSetpoint(RotateShooterConstants.kInitDegree);
    upGoalRate = ShooterConstants.kSpeakerShooterRate[0];
    downGoalRate = ShooterConstants.kSpeakerShooterRate[1];
    final double upMotorVoltage = upMotorFeedForwardController.calculate(upGoalRate)
        + rateShooterPID.calculate(getUpEncoderRate(), upGoalRate);
    final double downMotorVoltage = downMotorFeedForwardController.calculate(downGoalRate)
        + rateShooterPID.calculate(getDownEncoderRate(), downGoalRate);
    setUpMotorVoltage(upMotorVoltage);
    setDownMotorVoltage(downMotorVoltage);
    shootMode = 1;
  }

  public Command setInitRateControlCmd() {
    Command cmd = run(this::setInitRateControl);
    return cmd;
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
    upShooterMotor.set(VictorSPXControlMode.PercentOutput, power);
  }

  /**
   * Set up motor voltage, stop the motor if maximum/minimum power exceeded.
   * 
   * @param power
   */
  private void setDownMotor(double power) {
    downShooterMotor.set(VictorSPXControlMode.PercentOutput, power);
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

    SmartDashboard.putData(rotatePID);
    SmartDashboard.putNumber("encoderDegree", getAngle());
    SmartDashboard.putNumber("upMotorVoltage", upShooterMotor.getMotorOutputVoltage());
    SmartDashboard.putNumber("downMotorVoltage", downShooterMotor.getMotorOutputVoltage());
    SmartDashboard.putNumber("rotateSetpoint", setPoint);
  }

    public Command setManualVoltageCmd(double voltage) {
    Command cmd = runOnce(() -> setManualVoltage(voltage));
    cmd.setName("SetManualVoltageCmd");
    return cmd;
  }

  public Command transportModeCmd(){
    Command cmd = run(this::transportMode);
    return cmd;
  }

  // public Command shootRateControlModeCmd() {
  //   Command cmd = runOnce(this::shootRateControlMode);
  //   return cmd;
  // }

  public Command aimControlCmd() {
    Command cmd = runEnd(this::aimControl, this::stopAllMotor);
    return cmd;
  }
  // public Command shootRateControlCmd() {
  //   Command cmd = runEnd(this::shootRateControlMode, this::stopAllMotor);
  //   return cmd;
  // }
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
