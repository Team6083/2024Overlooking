// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants.IntakeConstants;
import frc.robot.subsystems.IntakeSubsystem;

public class TimeStopIntakeCmd extends Command {
  private final IntakeSubsystem intakeSubsystem;
  private Timer timer;
  /** Creates a new TimeStopIntake. */
  public TimeStopIntakeCmd(IntakeSubsystem intakeSubsystem,Timer timer) {
    this.intakeSubsystem = intakeSubsystem;
    this.timer = timer;
    addRequirements(this.intakeSubsystem);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    timer.start();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
      intakeSubsystem.setRotateMotorVoltage(IntakeConstants.kDownVoltage);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    intakeSubsystem.stopRotateIntakeMotor();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return new WaitCommand(IntakeConstants.kStopTime).isFinished();
  }
}
