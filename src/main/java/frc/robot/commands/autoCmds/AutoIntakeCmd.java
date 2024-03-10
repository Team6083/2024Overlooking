// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.autoCmds;

import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import frc.robot.commands.IntakeWithTransportCmd;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.TransportSubsystem;
import frc.robot.subsystems.drive.Drivebase;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class AutoIntakeCmd extends ParallelDeadlineGroup {
  /** Creates a new autoIntakeCmd. */
  public AutoIntakeCmd(Drivebase drivebase, TransportSubsystem transportSubsystem, IntakeSubsystem intakeSubsystem) {
    // Add the deadline command in the super() call. Add other commands using
    // addCommands().
    super(new IntakeWithTransportCmd(transportSubsystem, intakeSubsystem));
    addCommands(new AutoDriveForwardCmd(drivebase));
  }
}
