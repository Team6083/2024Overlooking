// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.autoCmds;

import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.commands.rotateShooterCmds.RotateShooterAutoControlCmd;
import frc.robot.subsystems.RotateShooterSubsystem;
import frc.robot.subsystems.drive.Drivebase;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class AutoAimCmd extends ParallelDeadlineGroup {
  /** Creates a new AutoAimCommand. */
  public AutoAimCmd(Drivebase drivebase, RotateShooterSubsystem rotateShooterSubsystem) {
    // Add the deadline command in the super() call. Add other commands using
    // addCommands().
    super(new WaitCommand(0.5));
    addCommands(
        Commands.run(() -> drivebase.drive(0, 0, drivebase.facingTag(0), true), drivebase),
        new RotateShooterAutoControlCmd(rotateShooterSubsystem));
  }
}
