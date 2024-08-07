// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Constants.AutoConstants;
import frc.robot.commands.driveControls.TagDriveCmd;

// import frc.robot.commands.autoCmds.PoseRotateShooterCmd;

// import frc.robot.commands.driveControls.NoteDriveCmd;

import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.TransportSubsystem;
import frc.robot.subsystems.drive.Drivebase;

public final class Autos {

public Command ShootCmd(Drivebase drivebase, ShooterSubsystem shooterSubsystem ,TransportSubsystem transportSubsystem,CommandXboxController maiController) {
        Command AutoAimControl = shooterSubsystem.speakerControlCmd(null, null);
        Command AutoTransport = transportSubsystem.transportIntakeCmd().withTimeout(0.5);
        Command AutoTag = new TagDriveCmd(drivebase, maiController);

        drivebase.resetPose(AutoConstants.middlePose2d);

        Command cmd = new ParallelDeadlineGroup(AutoTransport, AutoAimControl, AutoTag);

        return cmd;
    }

    // public static Command ShootandForwardCmd(Drivebase drivebase,
    // TransportSubsystem transportSubsystem, ShooterSubsystem shooterSubsystem,
    // CommandXboxController mainController) {

    // Command AutoAimControl = shooterSubsystem.speakerControlCmd(null,null);
    // Command AutoTransport =
    // transportSubsystem.transportIntakeCmd().withTimeout(0.5);
    // Command AutoNote = new NoteDriveCmd(drivebase,
    // mainController).withTimeout(0.5);
    // Command AutoTag = new TagDriveCmd(drivebase, mainController);

    // drivebase.resetPose(AutoConstants.leftPose2d);

    // Command cmd = new ParallelCommandGroup(AutoIntakeDown,
    // new ParallelDeadlineGroup(new WaitCommand(0.3).andThen(AutoTransport),
    // AutoAimControl));
    // cmd.andThen(drivebase.followPathCommand(AutoConstants.Amp1), AutoNote);
    // cmd.andThen(
    // new ParallelDeadlineGroup(drivebase.followPathCommand(AutoConstants.Amp2),
    // AutoIntakeWithTransport));
    // cmd.andThen(new ParallelDeadlineGroup(new
    // WaitCommand(0.3).andThen(AutoTransport), AutoAimControl, AutoTag));
    // cmd.andThen(drivebase.followPathCommand(AutoConstants.Amp3), AutoNote);

    // cmd.andThen(
    // new ParallelDeadlineGroup(drivebase.followPathCommand(AutoConstants.Amp4),
    // AutoIntakeWithTransport));
    // cmd.andThen(drivebase.followPathCommand(AutoConstants.Amp5));
    // cmd.andThen(new ParallelDeadlineGroup(new
    // WaitCommand(0.3).andThen(AutoTransport), AutoAimControl, AutoTag));

    // cmd.andThen(drivebase.followPathCommand(AutoConstants.Amp6));
    // cmd.andThen(
    // new ParallelDeadlineGroup(drivebase.followPathCommand(AutoConstants.Shoot,
    // AutoIntakeWithTransport)))
    
    // ;

    // return cmd;
    // }

    private Autos() {
        throw new UnsupportedOperationException("This is a utility class!");
    }

}