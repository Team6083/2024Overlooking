// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.intakeCmds;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.IntakeSubsystem;

public class IntakeDistanceCmd extends Command {
  /** Creates a new TeleopModeIntakeCmd. */
  private final IntakeSubsystem intakeSubsystem;
  private final boolean isGetNote;

  public IntakeDistanceCmd(IntakeSubsystem intakeSubsystem, boolean isGetNote) {
    this.intakeSubsystem = intakeSubsystem;
    this.isGetNote = isGetNote;
    addRequirements(this.intakeSubsystem);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    intakeSubsystem.stopMotor();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    intakeSubsystem.intakeCmd();
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    intakeSubsystem.stopMotor();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return isGetNote;
  }
}
