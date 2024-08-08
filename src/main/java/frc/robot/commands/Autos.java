// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathPlannerPath;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Constants.AutoConstants;
import frc.robot.commands.autoCmds.AutoTransportToShootCmd;

// import frc.robot.commands.driveControls.TagDriveCmd;

// import frc.robot.commands.autoCmds.PoseRotateShooterCmd;

// import frc.robot.commands.driveControls.NoteDriveCmd;

import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.TransportSubsystem;
import frc.robot.subsystems.drive.Drivebase;
import frc.robot.subsystems.IntakeSubsystem;;

public final class Autos {

    public static Command ShootCmd(Drivebase drivebase, ShooterSubsystem shooterSubsystem,
            TransportSubsystem transportSubsystem, CommandXboxController maiController,
            IntakeSubsystem intakeSubsystem) {

        Command AutoIntakeDown = new TimeStopIntakeCmd(intakeSubsystem).withTimeout(2.52);
        Command AutoTransport = transportSubsystem.transportIntakeCmd().withTimeout(0.5);
        Command AutoAimControl = shooterSubsystem.speakerControlCmd(() -> 0.0, () -> false);

        drivebase.resetPose(AutoConstants.middlePose2d);

        // Command cmd = new ParallelCommandGroup(AutoIntakeDown,
        // new ParallelDeadlineGroup(new WaitCommand(0.3).andThen(transportToShootCmd),
        // autoTransportToShootCmd));

        Command cmd = new ParallelCommandGroup(AutoIntakeDown,
                new ParallelDeadlineGroup(new WaitCommand(0.3).andThen(AutoTransport),
                        AutoAimControl));

        return cmd;
    }

    public static Command ShootandForwardCmd(Drivebase drivebase,
            TransportSubsystem transportSubsystem, ShooterSubsystem shooterSubsystem,
            CommandXboxController mainController, IntakeSubsystem intakeSubsystem) {

        Command AutoAimControl = shooterSubsystem.speakerControlCmd(()->0D, ()-> false);
        Command AutoTransport = transportSubsystem.transportIntakeCmd().withTimeout(0.5);
        Command AutoIntakeDown = new TimeStopIntakeCmd(intakeSubsystem).withTimeout(2.52);

        drivebase.resetPose(AutoConstants.leftPose2d);

        PathPlannerPath path = PathPlannerPath.fromPathFile(AutoConstants.ShootandForward);

        Command cmd = new ParallelCommandGroup(AutoIntakeDown,
                new ParallelDeadlineGroup(new WaitCommand(0.3).andThen(AutoTransport),
                        AutoAimControl));
        cmd.andThen (AutoBuilder.followPath(path));
        
        return cmd;
    }

    public static Command MiddleCmd(Drivebase drivebase,
            TransportSubsystem transportSubsystem, ShooterSubsystem shooterSubsystem,
            CommandXboxController maiController, IntakeSubsystem intakeSubsystem,IntakeWithTransportCmd intakeWithTransportCmd) {
        Command AutoAimControl = shooterSubsystem.speakerControlCmd(()->0D, ()->false);
        Command AutoTransport = transportSubsystem.transportIntakeCmd().withTimeout(0.5);
        Command AutoIntakeDown = new TimeStopIntakeCmd(intakeSubsystem).withTimeout(2.52);
        Command AutoIntakeWithTransport = new IntakeWithTransportCmd(transportSubsystem, intakeSubsystem);
        drivebase.resetPose(AutoConstants.leftPose2d);

        Command cmd = new ParallelCommandGroup(AutoIntakeDown,
                new ParallelDeadlineGroup(new WaitCommand(0.3).andThen(AutoTransport),
                        AutoAimControl));

     new ParallelDeadlineGroup(drivebase.followPathCommand("AutoConstants.Amp2"), AutoIntakeWithTransport);                
      cmd.andThen(new ParallelDeadlineGroup(new WaitCommand(0.3).andThen(AutoTransport),
                        AutoAimControl));
        return cmd;
    }


    private Autos() {
        throw new UnsupportedOperationException("This is a utility class!");
    }

}
