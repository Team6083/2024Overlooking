// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Constants.DrivebaseConstants;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.drive.Drivebase;

public class AdjustShooterAngleManual extends Command {
  /** Creates a new AdjustShooterAngleManual. */
  private final CommandXboxController main;
  private final ShooterSubsystem shooterSubsystem;
  private double adjust;

  public AdjustShooterAngleManual(CommandXboxController main, ShooterSubsystem shooterSubsystem) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.shooterSubsystem = shooterSubsystem;
    this.main = main;
    addRequirements(this.shooterSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    adjust = 3*main.getRawAxis(0);
    shooterSubsystem.AdjustShooterAngleManual(adjust);
    }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
