// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.RotateShooterConstants;
import frc.robot.subsystems.visionProcessing.TagTracking;

public class RotateShooterSubsystem extends SubsystemBase {

  /** Creates a new RiseShooterSubsytem. */
  private final CANSparkMax rotateMotor;
  private final DutyCycleEncoder rotateEncoder;
  private final PIDController rotatePID;
  private double rotateDegreeError = 0.0;
  private final PowerDistributionSubsystem powerDistributionSubsystem;
  private final TagTracking tagTracking;
  private int mode = 4;
  private boolean isMaunal = false;
  private double manualVoltage = 0;

  public RotateShooterSubsystem(PowerDistributionSubsystem powerDistributionSubsystem,
      TagTracking tagTracking) {
    rotateMotor = new CANSparkMax(RotateShooterConstants.kRotateShooterChannel, MotorType.kBrushless);

    rotateEncoder = new DutyCycleEncoder(RotateShooterConstants.kEncoderChannel);

    rotatePID = new PIDController(RotateShooterConstants.kP, RotateShooterConstants.kI, RotateShooterConstants.kD);

    rotateMotor.setInverted(RotateShooterConstants.kRotateShooterInverted);
    this.powerDistributionSubsystem = powerDistributionSubsystem;
    this.tagTracking = tagTracking;
    rotatePID.enableContinuousInput(-180.0, 180.0);
    setSetpoint(RotateShooterConstants.kInitDegree);
  }

  public void setManualControl() {
    setMotor(manualVoltage);
    rotatePID.setSetpoint(getAngle());
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

  public void setPIDControl() {
    double rotateVoltage = rotatePID.calculate(getAngle());
    double modifiedRotateVoltage = rotateVoltage;
    if (Math.abs(modifiedRotateVoltage) > RotateShooterConstants.kRotateVoltLimit) {
      modifiedRotateVoltage = RotateShooterConstants.kRotateVoltLimit * (rotateVoltage > 0 ? 1 : -1);
    }
    setMotor(modifiedRotateVoltage);
  }

  public double getAngle() {
    double degree = (RotateShooterConstants.kEncoderInverted ? -1.0 : 1.0)
        * ((rotateEncoder.getAbsolutePosition() * 360.0) - 120.0);
    return degree;
  }

  public double getAimDegree(double currentDegree) {
    if (tagTracking.getTv() == 1 && tagTracking.getTID() != 3.0
        && tagTracking.getTID() != 8.0) {
      double speakerToShooterHeight = RotateShooterConstants.kSpeakerHeight - RotateShooterConstants.kShooterHeight;
      double degree = Math.toDegrees(Math.atan(speakerToShooterHeight / tagTracking.getHorizontalDistanceByCT()));
      return degree;
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

  /**
   * @param 1 aim degree
   * @param 2 carry degree
   * @param 3 init degree
   */
  public void setModeSetpoint() {
    switch (mode) {
      case 1:
        setSetpoint(getAimDegree(getSetpoint()));
        break;
      case 2:
        setSetpoint(RotateShooterConstants.kCarryDegree);
        break;
      case 3:
        setSetpoint(RotateShooterConstants.kInitDegree);
      default:
        break;
    }
  }

  public void setManualVoltage(double voltage) {
    manualVoltage = voltage;
  }

  public Command setManualVoltageCmd(double voltage) {
    Command cmd = runOnce(() -> setManualVoltage(voltage));
    cmd.setName("SetManualVoltageCmd");
    return cmd;
  }

  public void changeMaunalMode(boolean isManual) {
    this.isMaunal = isManual;
  }

  /**
   * Reverse current manual mode.
   * @param mode
   * @return changeManualModeCmd
   */
  public Command changeMaunalModeCmd(boolean mode) {
    Command cmd = runOnce(() -> changeMaunalMode(mode));
    cmd.setName("changeMaunalModeCmd");
    return cmd;
  }

  /**
   * Set shooter mode.
   * @param mode
   */
  public void setMode(int mode) {
    this.mode = mode;
  }

  /**
   * 
   * @param mode
   * @return
   */
  public Command setModeCmd(int mode) {
    Command cmd = runOnce(() -> setMode(mode));
    cmd.setName("setModeCmd");
    return cmd;
  }

  public Command addErrorCmd(double error) {
    Command cmd = Commands.run(
        () -> addError(error), this);
    cmd.setName("addErrorCmd");
    return cmd;
  }

  public Command setAutoAim() {
    Command cmd = Commands.runOnce(
        () -> setSetpoint(getAimDegree(getSetpoint())), this);
    cmd.setName("autoAimCmd");
    return cmd;
  }

  public void addError(double error) {
    rotateDegreeError = error * RotateShooterConstants.kRotateDegreeErrorPoint;
  }

  public void resetEncoder() {
    rotateEncoder.reset();
  }

  public void resetSetpoint() {
    rotatePID.setSetpoint(0);
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

  private void changeRotateMode() {
    if (!isMaunal) {
      setModeSetpoint();
    } else {
      setManualControl();
    }
  }

  @Override
  public void periodic() {
    changeRotateMode();
    setPIDControl();
    SmartDashboard.putData("rotate_PID", rotatePID);
    SmartDashboard.putNumber("encoderDegree", getAngle());
  }

  public Command setInherentSetpoint() {
    Command cmd = Commands.run(
        () -> setSetpoint(RotateShooterConstants.kInitDegree),
        this);
    cmd.setName("setInherentSetpoint");
    return cmd;
  }
}
