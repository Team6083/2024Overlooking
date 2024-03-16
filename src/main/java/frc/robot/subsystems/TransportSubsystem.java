// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import com.revrobotics.Rev2mDistanceSensor.Port;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.TransportConstants;
import frc.robot.lib.DistanceSensor;
import frc.robot.lib.DistanceSensorInterface;
import frc.robot.lib.SimDistanceSensor;

public class TransportSubsystem extends SubsystemBase {
  /** Creates a new TransportSubsystem. */
  private final CANSparkMax transportMotor;
  private DistanceSensorInterface distanceSensor;
  private final PowerDistributionSubsystem powerDistributionSubsystem;

  public TransportSubsystem(PowerDistributionSubsystem powerDistribution) {

    transportMotor = new CANSparkMax(TransportConstants.kTransportChannel, MotorType.kBrushless);
    transportMotor.setInverted(TransportConstants.kTransportInverted);
    try {
      distanceSensor = new DistanceSensor(Port.kOnboard);

    } catch (Exception e) {
      distanceSensor = new SimDistanceSensor();
      System.out.println("distance sensor not detected");
    }

    distanceSensor.setAutomaticMode(true);
    this.powerDistributionSubsystem = powerDistribution;
  }

  /**
   * Transport from intake to shooter.
   */
  public void setTransport() {
    setVoltage(TransportConstants.kTransVoltage);
  }

  /**
   * Transport from shooter to intake.
   */
  public void setReTransport() {
    setVoltage(TransportConstants.kReTransVoltage);
  }

  /**
   * Stop transporting.
   */
  public void stopMotor() {
    transportMotor.set(0);
  }

  /**
   * Detect note. Returns true if note is detected.
   * 
   * @return boolean
   */
  public boolean isGetNote() {
    if (distanceSensor.isGetTarget()) {
      return distanceSensor.getTargetDistance() <= TransportConstants.kDistanceRange && distanceSensor.getTargetDistance() > 0;
    }
    return false;
  }

  /**
   * Set motor voltage. Stop motor if maximum/minimum voltage isexceeded.
   * 
   * @param voltage
   */
  private void setVoltage(double voltage) {
    if (powerDistributionSubsystem.isTransportOverCurrent()) {
      stopMotor();
      return;
    }
    transportMotor.setVoltage(voltage);
  }

  @Override
  public void periodic() {
    SmartDashboard.putNumber("distanceSensorDistance", distanceSensor.getTargetDistance());
    SmartDashboard.putBoolean("transportIsGetNote", isGetNote());
  }

  /**
   * Command of transporting to shooter then stop motor.
   * 
   * @return transportIntakeCmd
   */
  public Command transportIntakeCmd() {
    return this.runEnd(this::setTransport, this::stopMotor);
  }

  /**
   * Command of transporting to intake then stop motor.
   * 
   * @return reTransportIntakeCmd
   */
  public Command reTransportIntakeCmd() {
    return this.runEnd(this::setReTransport, this::stopMotor);
  }
}
