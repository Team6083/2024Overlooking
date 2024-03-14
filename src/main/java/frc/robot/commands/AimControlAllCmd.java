// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import frc.robot.Constants.ShooterConstants;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.drive.Drivebase;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class AimControlAllCmd extends ParallelCommandGroup {
  /** Creates a new aimControlAll. */
  public AimControlAllCmd(Drivebase drivebase, ShooterSubsystem shooterSubsystem) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(drivebase.tagTracking2Cmd(), shooterSubsystem.setAdjustAngleByTagCommand(), shooterSubsystem.shootRateControlModeCmd());
  }
}
