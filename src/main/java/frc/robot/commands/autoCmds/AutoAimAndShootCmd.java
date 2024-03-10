// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.autoCmds;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import frc.robot.subsystems.RotateShooterSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.TransportSubsystem;
import frc.robot.subsystems.drive.Drivebase;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class AutoAimAndShootCmd extends ParallelCommandGroup {
  /** Creates a new AutoAimAndShootCmd. */
  public AutoAimAndShootCmd(Drivebase drivebase, RotateShooterSubsystem rotateShooterSubsystem,
      ShooterSubsystem shooterSubsystem, TransportSubsystem transportSubsystem) {
    addCommands(
        new AutoAimCmd(drivebase, rotateShooterSubsystem),
        new AutoTransportShootCmd(drivebase, shooterSubsystem, transportSubsystem));
  }
}
