// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.controllerCmds;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.DrivebaseConstants;
import frc.robot.subsystems.drive.Drivebase;

public class DrivebaseDefaultSpeedCmd extends Command {
  Drivebase drivebase;

  /** Creates a new DrivebaseDefaultSpeedCmd. */
  public DrivebaseDefaultSpeedCmd(Drivebase drivebase) {
    this.drivebase = drivebase;
    addRequirements(this.drivebase);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    drivebase.setMagnification(DrivebaseConstants.kDefaultMagnification);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    drivebase.setMagnification(DrivebaseConstants.kDefaultMagnification);
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
