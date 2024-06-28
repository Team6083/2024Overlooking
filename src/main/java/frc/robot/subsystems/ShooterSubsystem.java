// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.function.Supplier;

import com.ctre.phoenix.motorcontrol.VictorSPXControlMode;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;
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
  // rotate shooter
  private final TagTracking tagTracking;
  private final CANSparkMax rotateMotor;
  private final DutyCycleEncoder rotateEncoder;
  private final PIDController rotatePID;

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
    rotateEncoder = new DutyCycleEncoder(RotateShooterConstants.kRotateEncoderChannel);
    rotatePID = new PIDController(RotateShooterConstants.kP, RotateShooterConstants.kI, RotateShooterConstants.kD);
    this.tagTracking = tagTracking;
    rotatePID.setSetpoint(RotateShooterConstants.kInitDegree);
  }

  public void stopRotateMotor() {
    rotateMotor.setVoltage(0.0);
  }

  public void setRotateMotor(double voltage) {
    rotateMotor.setVoltage(voltage);
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

  private int hasExceedPhysicalLimit(double angle) {
    return (angle < RotateShooterConstants.kRotateAngleMin ? -1
        : (angle > RotateShooterConstants.kRotateAngleMax ? 1 : 0));
  }

  private double getSetpoint() {
    return rotatePID.getSetpoint();
  }

  private void setSetpoint(double setpoint) {
    double currentSetpoint = getSetpoint();
    if (hasExceedPhysicalLimit(currentSetpoint) != 0) {
      return;
    }
    rotatePID.reset();
    double rotateDegree = setpoint;
    if (hasExceedPhysicalLimit(rotateDegree) == -1) {
      rotateDegree = RotateShooterConstants.kRotateAngleMin;
    } else if (hasExceedPhysicalLimit(rotateDegree) == 1) {
      rotateDegree = RotateShooterConstants.kRotateAngleMax;
    }
    rotatePID.setSetpoint(rotateDegree);
  }

  private void setPIDControl() {
    double rotateVoltage = rotatePID.calculate(getAngle());
    double modifiedRotateVoltage = rotateVoltage;
    if (Math.abs(modifiedRotateVoltage) > RotateShooterConstants.kRotateVoltLimit) {
      modifiedRotateVoltage = RotateShooterConstants.kRotateVoltLimit * (rotateVoltage > 0 ? 1 : -1);
    }
    setRotateMotor(modifiedRotateVoltage);
  }

  private double getAngle() {
    double degree = (RotateShooterConstants.kEncoderInverted ? -1.0 : 1.0)
        * ((rotateEncoder.getAbsolutePosition() * 360.0) - RotateShooterConstants.kRotateOffset
            - RotateShooterConstants.kShooterOffset)
        % 360.0;
    if (Math.abs(degree) > 180) {
      degree -= 360;
    }
    return degree;
  }

  private double getSpeakerDegree(double currentDegree, Supplier<Double> manualOffsetSupplier) {
    if (tagTracking.getTv() == 1 && tagTracking.getHorDistanceByCal() > 1.1) {
      double horizonDistance = tagTracking.getHorizontalDistanceByCT();
      double speakerToShooterHeight = RotateShooterConstants.kSpeakerHeight - RotateShooterConstants.kShooterHeight;
      double degree = Math.toDegrees(Math.atan(speakerToShooterHeight / horizonDistance));
      return degree + 6.0 * horizonDistance / 3.6 + manualOffsetSupplier.get();
    }
    return currentDegree;
  }

  /**
   * Stop both up and down shooter motor.
   */
  private void stopAllMotor() {
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

  private void speakerControl(Supplier<Double> manualOffsetSupplier, Supplier<Boolean> isManualSetpointSupplier) {
    if (isManualSetpointSupplier.get() == null || !isManualSetpointSupplier.get()) {
      var calculatedSetpoint = getSpeakerDegree(getSetpoint(), manualOffsetSupplier);
      setSetpoint(calculatedSetpoint);
    }
    double upGoalRate = ShooterConstants.kSpeakerShooterRate[0];
    double downGoalRate = ShooterConstants.kSpeakerShooterRate[1];
    final double upMotorVoltage = upMotorFeedForwardController.calculate(upGoalRate)
        + rateShooterPID.calculate(getUpEncoderRate(), upGoalRate);
    final double downMotorVoltage = downMotorFeedForwardController.calculate(downGoalRate)
        + rateShooterPID.calculate(getDownEncoderRate(), downGoalRate);
    setUpMotorVoltage(upMotorVoltage);
    setDownMotorVoltage(downMotorVoltage);
    setShootMode(1);
  }

  private void carryControl(Supplier<Boolean> isManualSetpointSupplier) {
    if (isManualSetpointSupplier.get() == null || !isManualSetpointSupplier.get()) {
      setSetpoint(RotateShooterConstants.kCarryDegree);
    }
    double upGoalRate = ShooterConstants.kCarryShooterRate[0];
    double downGoalRate = ShooterConstants.kCarryShooterRate[1];
    final double upMotorVoltage = upMotorFeedForwardController.calculate(upGoalRate)
        + rateShooterPID.calculate(getUpEncoderRate(), upGoalRate);
    final double downMotorVoltage = downMotorFeedForwardController.calculate(downGoalRate)
        + rateShooterPID.calculate(getDownEncoderRate(), downGoalRate);
    setUpMotorVoltage(upMotorVoltage);
    setDownMotorVoltage(downMotorVoltage);
    setShootMode(2);
  }

  private void ampControl(Supplier<Boolean> isManualSetpointSupplier) {
    if (isManualSetpointSupplier.get() == null || !isManualSetpointSupplier.get()) {
      setSetpoint(RotateShooterConstants.kAmpDegree);
    }
    double upGoalRate = ShooterConstants.kAmpShooterRate[0];
    double downGoalRate = ShooterConstants.kAmpShooterRate[1];
    final double upMotorVoltage = upMotorFeedForwardController.calculate(upGoalRate)
        + rateShooterPID.calculate(getUpEncoderRate(), upGoalRate);
    final double downMotorVoltage = downMotorFeedForwardController.calculate(downGoalRate)
        + rateShooterPID.calculate(getDownEncoderRate(), downGoalRate);
    SmartDashboard.putNumber("upgoalrate", upGoalRate);
    SmartDashboard.putNumber("downgoalrate", downGoalRate);
    setUpMotorVoltage(upMotorVoltage);
    setDownMotorVoltage(downMotorVoltage);
    setShootMode(3);
  }

  private void manualControl(Supplier<Integer> manualMode) {
    if (manualMode.get() == null) {
      return;
    }
    setRotateMotor(manualMode.get() == 0 ? RotateShooterConstants.kManualVoltage
        : (manualMode.get() == 180 ? -RotateShooterConstants.kManualVoltage : 0));
    rotatePID.setSetpoint(getAngle());
  }

  public Command manualControlCmd(Supplier<Integer> manualMode) {
    Command cmd = run(() -> manualControl(manualMode));
    cmd.setName("ManualControlCmd");
    return cmd;
  }

  private void initControl() {
    setSetpoint(RotateShooterConstants.kInitDegree);
  }

  public Command initControlCmd() {
    Command cmd = run(this::initControl);
    cmd.setName("InitControlCmd");
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
    double[] shooterRate;

    switch (shootMode) {
      case 1:
        shooterRate = ShooterConstants.kSpeakerShooterRate;
        break;
      case 2:
        shooterRate = ShooterConstants.kCarryShooterRate;
        break;
      case 3:
        shooterRate = ShooterConstants.kAmpShooterRate;
        break;
      default:
        return false;
    }

    return (getUpEncoderRate() >= shooterRate[0] - ShooterConstants.kShooterRateOffset
        && getDownEncoderRate() >= shooterRate[1] - ShooterConstants.kShooterRateOffset);
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
    SmartDashboard.putNumber("shootingUpMotorRate", getUpEncoderRate());
    SmartDashboard.putNumber("shootingDownMotorRate", getDownEncoderRate());
    SmartDashboard.putBoolean("shootingIsEnoughRate", isEnoughRate());
    SmartDashboard.putNumber("shootingRateMode", shootMode);
    SmartDashboard.putData(rotatePID);
    SmartDashboard.putNumber("rotateDegree", getAngle());
    SmartDashboard.putNumber("rotateUpMotorVoltage", upShooterMotor.getMotorOutputVoltage());
    SmartDashboard.putNumber("rotateDownMotorVoltage", downShooterMotor.getMotorOutputVoltage());
  }

  public Command speakerControlCmd(Supplier<Double> joystickManualOffsetSupplier,
      Supplier<Boolean> isManualSetpointSupplier) {
    Command cmd = runEnd(() -> speakerControl(joystickManualOffsetSupplier, isManualSetpointSupplier),
        this::stopAllMotor);
    cmd.setName("SpeakerControlCmd");
    return cmd;
  }

  public Command carryControlCmd(Supplier<Boolean> isManualSetpointSupplier) {
    Command cmd = runEnd(() -> carryControl(isManualSetpointSupplier), this::stopAllMotor);
    cmd.setName("CarryControlCmd");
    return cmd;
  }

  public Command ampControlCmd(Supplier<Boolean> isManualSetpointSupplier) {
    Command cmd = runEnd(() -> ampControl(isManualSetpointSupplier), this::stopAllMotor);
    cmd.setName("AmpControlCmd");
    return cmd;
  }

  public Command stopAllMotorCmd() {
    Command cmd = runOnce(() -> stopAllMotor());
    cmd.setName("StopAllMotor");
    return cmd;
  }
}
