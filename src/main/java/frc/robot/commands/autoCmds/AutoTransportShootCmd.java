// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.autoCmds;

import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.commands.transportCmds.TransportShootCmd;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.TransportSubsystem;
import frc.robot.subsystems.drive.Drivebase;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class AutoTransportShootCmd extends ParallelDeadlineGroup {
  /** Creates a new autoAimAndShootCmd. */
  public AutoTransportShootCmd(Drivebase drivebase, ShooterSubsystem shooterSubsystem,
      TransportSubsystem transportSubsystem) {
    super(new WaitCommand(1));
    addCommands(new TransportShootCmd(transportSubsystem, shooterSubsystem.isEnoughRate(0))
        .onlyWhile(() -> shooterSubsystem.isEnoughRate(0)));
  }
}
