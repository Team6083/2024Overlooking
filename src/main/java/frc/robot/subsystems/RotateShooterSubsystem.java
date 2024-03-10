// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.util.sendable.SendableBuilder;
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
  private final TagTracking apriltagTracking;
  private int switchMode = 4;
  private boolean isMaunal = false;
  // private final SparkMaxRelativeEncoder riseEncoderSPX;

  public RotateShooterSubsystem(PowerDistributionSubsystem powerDistributionSubsystem,
      TagTracking aprilTagTracking) {
    rotateMotor = new CANSparkMax(RotateShooterConstants.kRotateShooterChannel, MotorType.kBrushless);

    rotateEncoder = new DutyCycleEncoder(RotateShooterConstants.kEncoderChannel);

    rotatePID = new PIDController(RotateShooterConstants.kP, RotateShooterConstants.kI, RotateShooterConstants.kD);

    rotateMotor.setInverted(RotateShooterConstants.kRotateShooterInverted);
    this.powerDistributionSubsystem = powerDistributionSubsystem;
    this.apriltagTracking = aprilTagTracking;
    rotatePID.enableContinuousInput(-180.0, 180.0);
    setSetpoint(RotateShooterConstants.kInitDegree);
  }

  public void setManualControl(double RotateSpeed) {
    setMotor(RotateSpeed);
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
    if (apriltagTracking.getTv() == 1 && apriltagTracking.getTID() != 3.0
        && apriltagTracking.getTID() != 8.0) {
      double speakerToShooterHeight = RotateShooterConstants.kSpeakerHeight - RotateShooterConstants.kShooterHeight;
      double degree = Math.toDegrees(Math.atan(speakerToShooterHeight / apriltagTracking.getHorizontalDistanceByCT()));
      return degree;
    }
    return currentDegree;
  }

  public double getAMPDegree(double currentDegree) {
    if (apriltagTracking.getTv() == 1) {
      double ampToShooterHeight = RotateShooterConstants.kAMPHeight - RotateShooterConstants.kShooterHeight;
      double degree = Math.toDegrees(Math.atan(ampToShooterHeight / apriltagTracking.getHorizontalDistanceByCT()));
      return degree;
    }
    return currentDegree;
  }

  public void switchMode() {
    switch (switchMode) {
      //speakerDegree
      case 1:
        setSetpoint(getAimDegree(getSetpoint()));
        break;
      //carryDegree:0
      case 2:
        setSetpoint(RotateShooterConstants.kCarryDegree);
      //initDegree:57
      case 3:
        setSetpoint(RotateShooterConstants.kInitDegree);
        break;
      default:
        break;
    }
  }

  public void changeMaunalMode(boolean mode){
    isMaunal = mode;
  }

  public Command changeMaunalModeCmd(boolean mode){
    Command cmd = runOnce(() -> changeMaunalMode(mode));
    cmd.setName("changeMaunalModeCmd");
    return cmd;
  }
  public void setMode(int mode){
    switchMode = mode;
  }


  public Command setModeCmd(int mode){
    Command cmd = runOnce(()-> setMode(mode));
    cmd.setName("setModeCmd");
    return cmd;
  }

  public Command addErrorCmd(double error) {
    Command cmd = Commands.run(
        () -> addError(error), this);
    cmd.setName("addErrorCmd");
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

  private void changeRotateMode(){
    if(!isMaunal){
      switchMode();
      setPIDControl();
    }

  }

  @Override
  public void periodic() {
    changeRotateMode();
    SmartDashboard.putData("rotate_PID", rotatePID);
    SmartDashboard.putNumber("encoderDegree", getAngle());
    SmartDashboard.putNumber("switchMode", switchMode);
  }

  @Override
  public void initSendable(SendableBuilder builder) {
    builder.setSmartDashboardType("RotateShooterSubsystem");
    builder.addDoubleProperty("rotateVoltage", () -> rotateMotor.get() * rotateMotor.getBusVoltage(), null);
    builder.addDoubleProperty("roateAngelDegree", () -> this.getAimDegree(getAngle()), null);
    rotatePID.initSendable(builder);
  }
}
