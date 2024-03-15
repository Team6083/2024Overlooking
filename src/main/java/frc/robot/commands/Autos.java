// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Constants.AutoConstants;
import frc.robot.commands.driveControls.NoteDriveCmd;
import frc.robot.commands.driveControls.TagDriveCmd;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.TransportSubsystem;
import frc.robot.subsystems.drive.Drivebase;

public final class Autos {

        public static Command oneNote(Drivebase drivebase, IntakeSubsystem intakeSubsystem,
                        TransportSubsystem transportSubsystem, ShooterSubsystem shooterSubsystem,
                        CommandXboxController mainController) {

                Command AutoIntakeDown = new TimeStopIntakeCmd(intakeSubsystem).withTimeout(2.52);
                // Command AutoIntakeWithTransport = new
                // IntakeWithTransportCmd(transportSubsystem, intakeSubsystem);
                Command AutoAimControl = shooterSubsystem.aimControlCmd();
                Command AutoTransport = transportSubsystem.transportIntakeCmd().withTimeout(0.5);
                // Command AutoNote = new NoteDriveCmd(drivebase,
                // mainController).withTimeout(0.5);
                // Command AutoTag = new TagDriveCmd(drivebase, mainController);

                Command cmd = new ParallelCommandGroup(AutoIntakeDown,
                                new ParallelDeadlineGroup(new WaitCommand(0.3).andThen(AutoTransport), AutoAimControl));

                return cmd;
        }

        public static Command blueAmp(Drivebase drivebase, IntakeSubsystem intakeSubsystem,
                        TransportSubsystem transportSubsystem, ShooterSubsystem shooterSubsystem,
                        CommandXboxController mainController) {
                Command AutoIntakeDown = new TimeStopIntakeCmd(intakeSubsystem).withTimeout(2.52);
                Command AutoIntakeWithTransport = new IntakeWithTransportCmd(transportSubsystem, intakeSubsystem);
                Command AutoAimControl = shooterSubsystem.aimControlCmd();
                Command AutoTransport = transportSubsystem.transportIntakeCmd().withTimeout(0.5);
                Command AutoNote = new NoteDriveCmd(drivebase, mainController).withTimeout(0.5);
                Command AutoTag = new TagDriveCmd(drivebase, mainController);

                drivebase.resetPose(AutoConstants.leftPose2d);

                Command cmd = new ParallelCommandGroup(AutoIntakeDown,
                                new ParallelDeadlineGroup(new WaitCommand(0.3).andThen(AutoTransport), AutoAimControl));
                cmd.andThen(drivebase.followPathCommand(AutoConstants.Amp1), AutoNote);
                cmd.andThen(
                                new ParallelDeadlineGroup(drivebase.followPathCommand(AutoConstants.Amp2),
                                                AutoIntakeWithTransport));
                cmd.andThen(new ParallelDeadlineGroup(new WaitCommand(0.3).andThen(AutoTransport), AutoAimControl,
                                AutoTag));
                cmd.andThen(drivebase.followPathCommand(AutoConstants.Amp3), AutoNote);

                cmd.andThen(
                                new ParallelDeadlineGroup(drivebase.followPathCommand(AutoConstants.Amp4),
                                                AutoIntakeWithTransport));
                cmd.andThen(drivebase.followPathCommand(AutoConstants.Amp5));
                cmd.andThen(new ParallelDeadlineGroup(new WaitCommand(0.3).andThen(AutoTransport), AutoAimControl,
                                AutoTag));

                cmd.andThen(drivebase.followPathCommand(AutoConstants.Amp6));
                cmd.andThen(
                                new ParallelDeadlineGroup(drivebase.followPathCommand(AutoConstants.Amp7),
                                                AutoIntakeWithTransport));

                return cmd;
        }

        public static Command redAmp(Drivebase drivebase, IntakeSubsystem intakeSubsystem,
                        TransportSubsystem transportSubsystem, ShooterSubsystem shooterSubsystem,
                        CommandXboxController mainController) {
                Command AutoIntakeDown = new TimeStopIntakeCmd(intakeSubsystem).withTimeout(2.52);
                Command AutoIntakeWithTransport = new IntakeWithTransportCmd(transportSubsystem, intakeSubsystem);
                Command AutoAimControl = shooterSubsystem.aimControlCmd();
                Command AutoTransport = transportSubsystem.transportIntakeCmd().withTimeout(0.5);
                Command AutoNote = new NoteDriveCmd(drivebase, mainController).withTimeout(0.5);
                Command AutoTag = new TagDriveCmd(drivebase, mainController);

                drivebase.resetPose(new Pose2d(15.78, 6.53, Rotation2d.fromDegrees(120)));

                Command cmd = new ParallelCommandGroup(AutoIntakeDown,
                                new ParallelDeadlineGroup(new WaitCommand(0.3).andThen(AutoTransport), AutoAimControl));
                cmd.andThen(drivebase.followPathCommand(AutoConstants.Amp1), AutoNote);
                cmd.andThen(
                                new ParallelDeadlineGroup(drivebase.followPathCommand(AutoConstants.Amp2),
                                                AutoIntakeWithTransport));
                cmd.andThen(new ParallelDeadlineGroup(new WaitCommand(0.3).andThen(AutoTransport), AutoAimControl,
                                AutoTag));
                cmd.andThen(drivebase.followPathCommand(AutoConstants.Amp3), AutoNote);

                cmd.andThen(
                                new ParallelDeadlineGroup(drivebase.followPathCommand(AutoConstants.Amp4),
                                                AutoIntakeWithTransport));
                cmd.andThen(drivebase.followPathCommand(AutoConstants.Amp5));
                cmd.andThen(new ParallelDeadlineGroup(new WaitCommand(0.3).andThen(AutoTransport), AutoAimControl,
                                AutoTag));

                cmd.andThen(drivebase.followPathCommand(AutoConstants.Amp6));
                cmd.andThen(
                                new ParallelDeadlineGroup(drivebase.followPathCommand(AutoConstants.Amp7),
                                                AutoIntakeWithTransport));

                return cmd;
        }

        public static Command blueMiddle(Drivebase drivebase, IntakeSubsystem intakeSubsystem,
                        TransportSubsystem transportSubsystem, ShooterSubsystem shooterSubsystem,
                        CommandXboxController mainController) {
                Command AutoIntakeDown = new TimeStopIntakeCmd(intakeSubsystem).withTimeout(2.52);
                Command AutoIntakeWithTransport = new IntakeWithTransportCmd(transportSubsystem, intakeSubsystem);
                Command AutoAimControl = shooterSubsystem.aimControlCmd();
                Command AutoTransport = transportSubsystem.transportIntakeCmd().withTimeout(0.5);
                Command AutoNote = new NoteDriveCmd(drivebase, mainController).withTimeout(0.5);
                Command AutoTag = new TagDriveCmd(drivebase, mainController);

                drivebase.resetPose(AutoConstants.middlePose2d);

                Command cmd = new ParallelCommandGroup(AutoIntakeDown,
                                new ParallelDeadlineGroup(new WaitCommand(0.3).andThen(AutoTransport), AutoAimControl));
                cmd.andThen(drivebase.followPathCommand(AutoConstants.Middle1), AutoNote);
                cmd.andThen(
                                new ParallelDeadlineGroup(drivebase.followPathCommand(AutoConstants.Middle2),
                                                AutoIntakeWithTransport));
                cmd.andThen(new ParallelDeadlineGroup(new WaitCommand(0.3).andThen(AutoTransport), AutoAimControl,
                                AutoTag));
                cmd.andThen(drivebase.followPathCommand(AutoConstants.Middle3), AutoNote);

                cmd.andThen(
                                new ParallelDeadlineGroup(drivebase.followPathCommand(AutoConstants.Middle4),
                                                AutoIntakeWithTransport));
                cmd.andThen(drivebase.followPathCommand(AutoConstants.Middle5));
                cmd.andThen(new ParallelDeadlineGroup(new WaitCommand(0.3).andThen(AutoTransport), AutoAimControl,
                                AutoTag));

                cmd.andThen(drivebase.followPathCommand(AutoConstants.Middle6));
                cmd.andThen(
                                new ParallelDeadlineGroup(drivebase.followPathCommand(AutoConstants.Middle7),
                                                AutoIntakeWithTransport));

                return cmd;
        }

        public static Command redMiddle(Drivebase drivebase, IntakeSubsystem intakeSubsystem,
                        TransportSubsystem transportSubsystem, ShooterSubsystem shooterSubsystem,
                        CommandXboxController mainController) {
                Command AutoIntakeDown = new TimeStopIntakeCmd(intakeSubsystem).withTimeout(2.52);
                Command AutoIntakeWithTransport = new IntakeWithTransportCmd(transportSubsystem, intakeSubsystem);
                Command AutoAimControl = shooterSubsystem.aimControlCmd();
                Command AutoTransport = transportSubsystem.transportIntakeCmd().withTimeout(0.5);
                Command AutoNote = new NoteDriveCmd(drivebase, mainController).withTimeout(0.5);
                Command AutoTag = new TagDriveCmd(drivebase, mainController);

                drivebase.resetPose(new Pose2d(16.54 - 1.24, 5.5, Rotation2d.fromDegrees(180)));

                Command cmd = new ParallelCommandGroup(AutoIntakeDown,
                                new ParallelDeadlineGroup(new WaitCommand(0.3).andThen(AutoTransport), AutoAimControl));
                cmd.andThen(drivebase.followPathCommand(AutoConstants.Middle1), AutoNote);
                cmd.andThen(
                                new ParallelDeadlineGroup(drivebase.followPathCommand(AutoConstants.Middle2),
                                                AutoIntakeWithTransport));
                cmd.andThen(new ParallelDeadlineGroup(new WaitCommand(0.3).andThen(AutoTransport), AutoAimControl,
                                AutoTag));
                cmd.andThen(drivebase.followPathCommand(AutoConstants.Middle3), AutoNote);

                cmd.andThen(
                                new ParallelDeadlineGroup(drivebase.followPathCommand(AutoConstants.Middle4),
                                                AutoIntakeWithTransport));
                cmd.andThen(drivebase.followPathCommand(AutoConstants.Middle5));
                cmd.andThen(new ParallelDeadlineGroup(new WaitCommand(0.3).andThen(AutoTransport), AutoAimControl,
                                AutoTag));

                cmd.andThen(drivebase.followPathCommand(AutoConstants.Middle6));
                cmd.andThen(
                                new ParallelDeadlineGroup(drivebase.followPathCommand(AutoConstants.Middle7),
                                                AutoIntakeWithTransport));

                return cmd;
        }

        // public static Command blueSource(Drivebase drivebase, IntakeSubsystem intakeSubsystem,
        //                 TransportSubsystem transportSubsystem, ShooterSubsystem shooterSubsystem,
        //                 CommandXboxController mainController) {
        //         Command AutoIntakeDown = new TimeStopIntakeCmd(intakeSubsystem).withTimeout(2.52);
        //         Command AutoIntakeWithTransport = new IntakeWithTransportCmd(transportSubsystem, intakeSubsystem);
        //         Command AutoAimControl = shooterSubsystem.aimControlCmd();
        //         Command AutoTransport = transportSubsystem.transportIntakeCmd().withTimeout(0.5);
        //         Command AutoNote = new NoteDriveCmd(drivebase, mainController).withTimeout(0.5);
        //         Command AutoTag = new TagDriveCmd(drivebase, mainController);


        //         Command cmd 
        //         return cmd;
        // }

        /*
         * public static Command autoOptimize(Drivebase drivebase,
         * ShooterSubsystem shooterSubsystem, TransportSubsystem transportSubsystem,
         * IntakeSubsystem intakeSubsystem,
         * String autoNumber, String initial) {
         * 
         * int length = autoNumber.length();
         * Command runPeriodicCommand = shooterSubsystem.setAutoAimCmd();
         * Command runAutoCommand = Commands.deadline(
         * new AutoTransportToShootCmd(transportSubsystem, shooterSubsystem),
         * shooterSubsystem.setSpeakerRateControlCmd());
         * char pre = '0';
         * boolean firstTime = true;
         * 
         * // initial setting
         * switch (initial) {
         * case "left":
         * drivebase.resetPose(AutoConstants.leftPose2d);
         * break;
         * case "middle":
         * drivebase.resetPose(AutoConstants.middlePose2d);
         * break;
         * case "right":
         * drivebase.resetPose(AutoConstants.rightPose2d);
         * break;
         * }
         * 
         * // execute auto
         * for (int i = 0; i < length; i++) {
         * char cur = autoNumber.charAt(i);
         * 
         * // move to note
         * if (pre == '0' || pre == '1' || pre == '2' || pre == '3') {
         * switch (cur) {
         * case '1':
         * runAutoCommand
         * .andThen(drivebase.pathFindingToPose(2.13, 6.68, 25.53,
         * AutoConstants.kMaxVelocity,
         * AutoConstants.kMaxAcceleration, AutoConstants.kMaxAngularVelocity,
         * AutoConstants.kMaxAngularAcceleration, 0, 0));
         * break;
         * case '2':
         * runAutoCommand.andThen(drivebase.pathFindingToPose(2.13, 5.5, 0,
         * AutoConstants.kMaxVelocity,
         * AutoConstants.kMaxAcceleration, AutoConstants.kMaxAngularVelocity,
         * AutoConstants.kMaxAngularAcceleration, 0, 0));
         * break;
         * case '3':
         * runAutoCommand
         * .andThen(drivebase.pathFindingToPose(2.13, 4.4, -29.16,
         * AutoConstants.kMaxVelocity,
         * AutoConstants.kMaxAcceleration, AutoConstants.kMaxAngularVelocity,
         * AutoConstants.kMaxAngularAcceleration, 0, 0));
         * break;
         * case '4':
         * runAutoCommand.andThen(drivebase.pathFindingThenFollowPath(AutoConstants.
         * RTSToNote4,
         * AutoConstants.kMaxVelocity, AutoConstants.kMaxAcceleration,
         * AutoConstants.kMaxAngularVelocity,
         * AutoConstants.kMaxAngularAcceleration, 0.0));
         * break;
         * case '5':
         * runAutoCommand.andThen(drivebase.pathFindingThenFollowPath(AutoConstants.
         * RTSToNote5,
         * AutoConstants.kMaxVelocity, AutoConstants.kMaxAcceleration,
         * AutoConstants.kMaxAngularVelocity,
         * AutoConstants.kMaxAngularAcceleration, 0.0));
         * break;
         * case '6':
         * runAutoCommand.andThen(drivebase.pathFindingThenFollowPath(AutoConstants.
         * RBSToNote6,
         * AutoConstants.kMaxVelocity, AutoConstants.kMaxAcceleration,
         * AutoConstants.kMaxAngularVelocity,
         * AutoConstants.kMaxAngularAcceleration, 0.0));
         * break;
         * case '7':
         * runAutoCommand.andThen(drivebase.pathFindingThenFollowPath(AutoConstants.
         * RBSToNote7,
         * AutoConstants.kMaxVelocity, AutoConstants.kMaxAcceleration,
         * AutoConstants.kMaxAngularVelocity,
         * AutoConstants.kMaxAngularAcceleration, 0.0));
         * break;
         * case '8':
         * runAutoCommand.andThen(drivebase.pathFindingThenFollowPath(AutoConstants.
         * RBSToNote8,
         * AutoConstants.kMaxVelocity, AutoConstants.kMaxAcceleration,
         * AutoConstants.kMaxAngularVelocity,
         * AutoConstants.kMaxAngularAcceleration, 0.0));
         * break;
         * }
         * 
         * } else if (pre == '4' || pre == '5' || pre == '6') {
         * switch (cur) {
         * case '1':
         * runAutoCommand.andThen(drivebase.followPathCommand(AutoConstants.RTSToNote1))
         * ;
         * break;
         * case '2':
         * runAutoCommand.andThen(drivebase.followPathCommand(AutoConstants.RTSToNote2))
         * ;
         * break;
         * case '3':
         * runAutoCommand.andThen(drivebase.followPathCommand(AutoConstants.RTSToNote3))
         * ;
         * break;
         * case '4':
         * runAutoCommand.andThen(drivebase.followPathCommand(AutoConstants.RTSToNote4))
         * ;
         * break;
         * case '5':
         * runAutoCommand.andThen(drivebase.followPathCommand(AutoConstants.RTSToNote5))
         * ;
         * break;
         * case '6':
         * runAutoCommand.andThen(drivebase.followPathCommand(AutoConstants.RTSToNote6))
         * ;
         * break;
         * case '7':
         * runAutoCommand.andThen(drivebase.followPathCommand(AutoConstants.RTSToNote7))
         * ;
         * break;
         * case '8':
         * runAutoCommand.andThen(drivebase.followPathCommand(AutoConstants.RTSToNote8))
         * ;
         * break;
         * }
         * } else if (pre == '7' || pre == '8') {
         * switch (cur) {
         * case '1':
         * runAutoCommand.andThen(drivebase.followPathCommand(AutoConstants.RBSToNote1))
         * ;
         * break;
         * case '2':
         * runAutoCommand.andThen(drivebase.followPathCommand(AutoConstants.RBSToNote2))
         * ;
         * break;
         * case '3':
         * runAutoCommand.andThen(drivebase.followPathCommand(AutoConstants.RBSToNote3))
         * ;
         * break;
         * case '4':
         * runAutoCommand.andThen(drivebase.followPathCommand(AutoConstants.RBSToNote4))
         * ;
         * break;
         * case '5':
         * runAutoCommand.andThen(drivebase.followPathCommand(AutoConstants.RBSToNote5))
         * ;
         * break;
         * case '6':
         * runAutoCommand.andThen(drivebase.followPathCommand(AutoConstants.RBSToNote6))
         * ;
         * break;
         * case '7':
         * runAutoCommand.andThen(drivebase.followPathCommand(AutoConstants.RBSToNote7))
         * ;
         * break;
         * case '8':
         * runAutoCommand.andThen(drivebase.followPathCommand(AutoConstants.RBSToNote8))
         * ;
         * break;
         * }
         * }
         * 
         * // whether rerun path or not
         * if (!drivebase.hasTargets()) {
         * if (firstTime) {
         * firstTime = false;
         * i--;
         * continue;
         * }
         * pre = cur;
         * firstTime = true;
         * }
         * 
         * // get note
         * runAutoCommand.andThen(new IntakeWithTransportCmd(transportSubsystem,
         * intakeSubsystem));
         * 
         * // move to shoot point and shoot
         * switch (cur) {
         * case '1':
         * runAutoCommand
         * .andThen(Commands.deadline(new AutoTransportToShootCmd(transportSubsystem,
         * shooterSubsystem),
         * shooterSubsystem.setSpeakerRateControlCmd()));
         * break;
         * case '2':
         * runAutoCommand
         * .andThen(Commands.deadline(new AutoTransportToShootCmd(transportSubsystem,
         * shooterSubsystem),
         * shooterSubsystem.setSpeakerRateControlCmd()));
         * case '3':
         * runAutoCommand
         * .andThen(Commands.deadline(new AutoTransportToShootCmd(transportSubsystem,
         * shooterSubsystem),
         * shooterSubsystem.setSpeakerRateControlCmd()));
         * break;
         * case '4':
         * runAutoCommand.andThen(drivebase.pathFindingThenFollowPath(AutoConstants.
         * topRelayToRTS,
         * AutoConstants.kMaxVelocity, AutoConstants.kMaxAcceleration,
         * AutoConstants.kMaxAngularVelocity,
         * AutoConstants.kMaxAngularAcceleration, 0.0))
         * .andThen(Commands.deadline(new AutoTransportToShootCmd(transportSubsystem,
         * shooterSubsystem),
         * shooterSubsystem.setSpeakerRateControlCmd()));
         * break;
         * case '5':
         * runAutoCommand.andThen(drivebase.pathFindingThenFollowPath(AutoConstants.
         * topRelayToRTS,
         * AutoConstants.kMaxVelocity, AutoConstants.kMaxAcceleration,
         * AutoConstants.kMaxAngularVelocity,
         * AutoConstants.kMaxAngularAcceleration, 0.0))
         * .andThen(Commands.deadline(new AutoTransportToShootCmd(transportSubsystem,
         * shooterSubsystem),
         * shooterSubsystem.setSpeakerRateControlCmd()));
         * break;
         * case '6':
         * runAutoCommand.andThen(drivebase.pathFindingThenFollowPath(AutoConstants.
         * topRelayToRTS,
         * AutoConstants.kMaxVelocity, AutoConstants.kMaxAcceleration,
         * AutoConstants.kMaxAngularVelocity,
         * AutoConstants.kMaxAngularAcceleration, 0.0))
         * .andThen(Commands.deadline(new AutoTransportToShootCmd(transportSubsystem,
         * shooterSubsystem),
         * shooterSubsystem.setSpeakerRateControlCmd()));
         * break;
         * case '7':
         * runAutoCommand.andThen(drivebase.pathFindingThenFollowPath(AutoConstants.
         * bottomRelayToRBS,
         * AutoConstants.kMaxVelocity, AutoConstants.kMaxAcceleration,
         * AutoConstants.kMaxAngularVelocity,
         * AutoConstants.kMaxAngularAcceleration, 0.0))
         * .andThen(Commands.deadline(new AutoTransportToShootCmd(transportSubsystem,
         * shooterSubsystem),
         * shooterSubsystem.setSpeakerRateControlCmd()));
         * break;
         * case '8':
         * runAutoCommand.andThen(drivebase.pathFindingThenFollowPath(AutoConstants.
         * bottomRelayToRBS,
         * AutoConstants.kMaxVelocity, AutoConstants.kMaxAcceleration,
         * AutoConstants.kMaxAngularVelocity,
         * AutoConstants.kMaxAngularAcceleration, 0.0))
         * .andThen(Commands.deadline(new AutoTransportToShootCmd(transportSubsystem,
         * shooterSubsystem),
         * shooterSubsystem.setSpeakerRateControlCmd()));
         * break;
         * }
         * }
         * 
         * return new ParallelCommandGroup(runAutoCommand, runPeriodicCommand);
         * }
         * 
         * public static Command autoWithOnlyPose(Drivebase drivebase,
         * ShooterSubsystem shooterSubsystem, TransportSubsystem transportSubsystem,
         * IntakeSubsystem intakeSubsystem,
         * String autoNumber, String initial) {
         * 
         * int length = autoNumber.length();
         * Command runPeriodicCommand = null;
         * Command runAutoCommand = new AutoTransportToShootCmd(transportSubsystem,
         * shooterSubsystem);
         * char pre = '0';
         * 
         * // initial setting
         * switch (initial) {
         * case "left":
         * drivebase.resetPose(AutoConstants.leftPose2d);
         * break;
         * case "middle":
         * drivebase.resetPose(AutoConstants.middlePose2d);
         * break;
         * case "right":
         * drivebase.resetPose(AutoConstants.rightPose2d);
         * break;
         * }
         * 
         * // execute auto
         * for (int i = 0; i < length; i++) {
         * char cur = autoNumber.charAt(i);
         * 
         * // move to note
         * if (pre == '0' || pre == '1' || pre == '2' || pre == '3') {
         * switch (cur) {
         * case '1':
         * runAutoCommand
         * .andThen(drivebase.pathFindingToPose(2.13, 6.68, 25.53,
         * AutoConstants.kMaxVelocity,
         * AutoConstants.kMaxAcceleration, AutoConstants.kMaxAngularVelocity,
         * AutoConstants.kMaxAngularAcceleration, 0, 0));
         * break;
         * case '2':
         * runAutoCommand.andThen(drivebase.pathFindingToPose(2.13, 5.5, 0,
         * AutoConstants.kMaxVelocity,
         * AutoConstants.kMaxAcceleration, AutoConstants.kMaxAngularVelocity,
         * AutoConstants.kMaxAngularAcceleration, 0, 0));
         * break;
         * case '3':
         * runAutoCommand
         * .andThen(drivebase.pathFindingToPose(2.13, 4.4, -29.16,
         * AutoConstants.kMaxVelocity,
         * AutoConstants.kMaxAcceleration, AutoConstants.kMaxAngularVelocity,
         * AutoConstants.kMaxAngularAcceleration, 0, 0));
         * break;
         * case '4':
         * runAutoCommand.andThen(drivebase.pathFindingThenFollowPath(AutoConstants.
         * RTSToNote4,
         * AutoConstants.kMaxVelocity, AutoConstants.kMaxAcceleration,
         * AutoConstants.kMaxAngularVelocity,
         * AutoConstants.kMaxAngularAcceleration, 0.0));
         * break;
         * case '5':
         * runAutoCommand.andThen(drivebase.pathFindingThenFollowPath(AutoConstants.
         * RTSToNote5,
         * AutoConstants.kMaxVelocity, AutoConstants.kMaxAcceleration,
         * AutoConstants.kMaxAngularVelocity,
         * AutoConstants.kMaxAngularAcceleration, 0.0));
         * break;
         * case '6':
         * runAutoCommand.andThen(drivebase.pathFindingThenFollowPath(AutoConstants.
         * RBSToNote6,
         * AutoConstants.kMaxVelocity, AutoConstants.kMaxAcceleration,
         * AutoConstants.kMaxAngularVelocity,
         * AutoConstants.kMaxAngularAcceleration, 0.0));
         * break;
         * case '7':
         * runAutoCommand.andThen(drivebase.pathFindingThenFollowPath(AutoConstants.
         * RBSToNote7,
         * AutoConstants.kMaxVelocity, AutoConstants.kMaxAcceleration,
         * AutoConstants.kMaxAngularVelocity,
         * AutoConstants.kMaxAngularAcceleration, 0.0));
         * break;
         * case '8':
         * runAutoCommand.andThen(drivebase.pathFindingThenFollowPath(AutoConstants.
         * RBSToNote8,
         * AutoConstants.kMaxVelocity, AutoConstants.kMaxAcceleration,
         * AutoConstants.kMaxAngularVelocity,
         * AutoConstants.kMaxAngularAcceleration, 0.0));
         * break;
         * }
         * 
         * } else if (pre == '4' || pre == '5' || pre == '6') {
         * switch (cur) {
         * case '1':
         * runAutoCommand.andThen(drivebase.followPathCommand(AutoConstants.RTSToNote1))
         * ;
         * break;
         * case '2':
         * runAutoCommand.andThen(drivebase.followPathCommand(AutoConstants.RTSToNote2))
         * ;
         * break;
         * case '3':
         * runAutoCommand.andThen(drivebase.followPathCommand(AutoConstants.RTSToNote3))
         * ;
         * break;
         * case '4':
         * runAutoCommand.andThen(drivebase.followPathCommand(AutoConstants.RTSToNote4))
         * ;
         * break;
         * case '5':
         * runAutoCommand.andThen(drivebase.followPathCommand(AutoConstants.RTSToNote5))
         * ;
         * break;
         * case '6':
         * runAutoCommand.andThen(drivebase.followPathCommand(AutoConstants.RTSToNote6))
         * ;
         * break;
         * case '7':
         * runAutoCommand.andThen(drivebase.followPathCommand(AutoConstants.RTSToNote7))
         * ;
         * break;
         * case '8':
         * runAutoCommand.andThen(drivebase.followPathCommand(AutoConstants.RTSToNote8))
         * ;
         * break;
         * }
         * } else if (pre == '7' || pre == '8') {
         * switch (cur) {
         * case '1':
         * runAutoCommand.andThen(drivebase.followPathCommand(AutoConstants.RBSToNote1))
         * ;
         * break;
         * case '2':
         * runAutoCommand.andThen(drivebase.followPathCommand(AutoConstants.RBSToNote2))
         * ;
         * break;
         * case '3':
         * runAutoCommand.andThen(drivebase.followPathCommand(AutoConstants.RBSToNote3))
         * ;
         * break;
         * case '4':
         * runAutoCommand.andThen(drivebase.followPathCommand(AutoConstants.RBSToNote4))
         * ;
         * break;
         * case '5':
         * runAutoCommand.andThen(drivebase.followPathCommand(AutoConstants.RBSToNote5))
         * ;
         * break;
         * case '6':
         * runAutoCommand.andThen(drivebase.followPathCommand(AutoConstants.RBSToNote6))
         * ;
         * break;
         * case '7':
         * runAutoCommand.andThen(drivebase.followPathCommand(AutoConstants.RBSToNote7))
         * ;
         * break;
         * case '8':
         * runAutoCommand.andThen(drivebase.followPathCommand(AutoConstants.RBSToNote8))
         * ;
         * break;
         * }
         * }
         * 
         * // get note
         * runAutoCommand.andThen(new IntakeWithTransportCmd(transportSubsystem,
         * intakeSubsystem));
         * 
         * // move to shoot point and shoot
         * switch (cur) {
         * case '1':
         * runAutoCommand
         * .andThen(new AutoTransportToShootCmd(transportSubsystem, shooterSubsystem));
         * break;
         * case '2':
         * runAutoCommand
         * .andThen(new AutoTransportToShootCmd(transportSubsystem, shooterSubsystem));
         * break;
         * case '3':
         * runAutoCommand
         * .andThen(new AutoTransportToShootCmd(transportSubsystem, shooterSubsystem));
         * break;
         * case '4':
         * runAutoCommand.andThen(drivebase.pathFindingThenFollowPath(AutoConstants.
         * topRelayToRTS,
         * AutoConstants.kMaxVelocity, AutoConstants.kMaxAcceleration,
         * AutoConstants.kMaxAngularVelocity,
         * AutoConstants.kMaxAngularAcceleration, 0.0))
         * .andThen(new AutoTransportToShootCmd(transportSubsystem, shooterSubsystem));
         * break;
         * case '5':
         * runAutoCommand.andThen(drivebase.pathFindingThenFollowPath(AutoConstants.
         * topRelayToRTS,
         * AutoConstants.kMaxVelocity, AutoConstants.kMaxAcceleration,
         * AutoConstants.kMaxAngularVelocity,
         * AutoConstants.kMaxAngularAcceleration, 0.0))
         * .andThen(new AutoTransportToShootCmd(transportSubsystem, shooterSubsystem));
         * break;
         * case '6':
         * runAutoCommand.andThen(drivebase.pathFindingThenFollowPath(AutoConstants.
         * topRelayToRTS,
         * AutoConstants.kMaxVelocity, AutoConstants.kMaxAcceleration,
         * AutoConstants.kMaxAngularVelocity,
         * AutoConstants.kMaxAngularAcceleration, 0.0))
         * .andThen(new AutoTransportToShootCmd(transportSubsystem, shooterSubsystem));
         * break;
         * case '7':
         * runAutoCommand.andThen(drivebase.pathFindingThenFollowPath(AutoConstants.
         * bottomRelayToRBS,
         * AutoConstants.kMaxVelocity, AutoConstants.kMaxAcceleration,
         * AutoConstants.kMaxAngularVelocity,
         * AutoConstants.kMaxAngularAcceleration, 0.0))
         * .andThen(new AutoTransportToShootCmd(transportSubsystem, shooterSubsystem));
         * break;
         * case '8':
         * runAutoCommand.andThen(drivebase.pathFindingThenFollowPath(AutoConstants.
         * bottomRelayToRBS,
         * AutoConstants.kMaxVelocity, AutoConstants.kMaxAcceleration,
         * AutoConstants.kMaxAngularVelocity,
         * AutoConstants.kMaxAngularAcceleration, 0.0))
         * .andThen(new AutoTransportToShootCmd(transportSubsystem, shooterSubsystem));
         * break;
         * }
         * }
         * 
         * return new ParallelCommandGroup(runPeriodicCommand, runAutoCommand);
         * }
         */
        private Autos() {
                throw new UnsupportedOperationException("This is a utility class!");
        }

}
