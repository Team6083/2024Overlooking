// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.autoCmds;

import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.TransportSubsystem;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class AutoTransportToShootCmd extends ParallelDeadlineGroup {
  /** Creates a new autoTransportToShootCmd. */
  public AutoTransportToShootCmd( TransportSubsystem transportSubsystem, ShooterSubsystem shooterSubsystem) {
    // Add the deadline command in the super() call. Add other commands using
    // addCommands().
    super(new WaitCommand(0.8));
    addCommands(shooterSubsystem.speakerControlCmd(null), new WaitCommand(0.3).andThen(transportSubsystem.transportIntakeCmd()));
  }
}
