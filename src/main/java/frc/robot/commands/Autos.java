// // Copyright (c) FIRST and other WPILib contributors.
// // Open Source Software; you can modify and/or share it under the terms of
// // the WPILib BSD license file in the root directory of this project.

// package frc.robot.commands;

// import edu.wpi.first.wpilibj2.command.Command;
// import edu.wpi.first.wpilibj2.command.Commands;
// import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
// import frc.robot.Constants.AutoConstants;
// import frc.robot.commands.autoCmds.AutoAimAndShootCmd;
// import frc.robot.commands.autoCmds.AutoIntakeCmd;
// import frc.robot.commands.autoCmds.AutoRotateShooterCmd;
// import frc.robot.commands.autoCmds.AutoTransportShootCmd;
// import frc.robot.commands.autoCmds.PoseRotateShooterCmd;
// import frc.robot.commands.autoCmds.PoseShootCmd;
// import frc.robot.subsystems.IntakeSubsystem;
// import frc.robot.subsystems.RotateShooterSubsystem;
// import frc.robot.subsystems.ShooterSubsystem;
// import frc.robot.subsystems.TransportSubsystem;
// import frc.robot.subsystems.drive.Drivebase;

// public final class Autos {

//   public static Command autoOptimize(Drivebase drivebase, RotateShooterSubsystem rotateShooterSubsystem,
//       ShooterSubsystem shooterSubsystem, TransportSubsystem transportSubsystem, IntakeSubsystem intakeSubsystem,
//       String autoNumber, String initial) {

//     int length = autoNumber.length();
//     Command runPeriodicCommand = new ParallelCommandGroup(
//         new AutoRotateShooterCmd(rotateShooterSubsystem),
//         shooterSubsystem.shootPIDRateCmd());
//     Command runAutoCommand = new AutoAimAndShootCmd(drivebase, rotateShooterSubsystem, shooterSubsystem,
//         transportSubsystem);
//     char pre = '0';
//     boolean firstTime = true;

//     // initial setting
//     switch (initial) {
//       case "left":
//         drivebase.resetPose(AutoConstants.leftPose2d);
//         break;
//       case "middle":
//         drivebase.resetPose(AutoConstants.middlePose2d);
//         break;
//       case "right":
//         drivebase.resetPose(AutoConstants.rightPose2d);
//         break;
//     }

//     // execute auto
//     for (int i = 0; i < length; i++) {
//       char cur = autoNumber.charAt(i);

//       // move to note
//       if (pre == '0' || pre == '1' || pre == '2' || pre == '3') {
//         switch (cur) {
//           case '1':
//             runAutoCommand.andThen(drivebase.pathFindingToPose(2.13, 6.68, 25.53, AutoConstants.kMaxVelocity,
//                 AutoConstants.kMaxAcceleration, AutoConstants.kMaxAngularVelocity,
//                 AutoConstants.kMaxAngularAcceleration, 0, 0));
//             break;
//           case '2':
//             runAutoCommand.andThen(drivebase.pathFindingToPose(2.13, 5.5, 0, AutoConstants.kMaxVelocity,
//                 AutoConstants.kMaxAcceleration, AutoConstants.kMaxAngularVelocity,
//                 AutoConstants.kMaxAngularAcceleration, 0, 0));
//             break;
//           case '3':
//             runAutoCommand.andThen(drivebase.pathFindingToPose(2.13, 4.4, -29.16, AutoConstants.kMaxVelocity,
//                 AutoConstants.kMaxAcceleration, AutoConstants.kMaxAngularVelocity,
//                 AutoConstants.kMaxAngularAcceleration, 0, 0));
//             break;
//           case '4':
//             runAutoCommand.andThen(drivebase.pathFindingThenFollowPath(AutoConstants.RTSToNote4,
//                 AutoConstants.kMaxVelocity, AutoConstants.kMaxAcceleration, AutoConstants.kMaxAngularVelocity,
//                 AutoConstants.kMaxAngularAcceleration, 0.0));
//             break;
//           case '5':
//             runAutoCommand.andThen(drivebase.pathFindingThenFollowPath(AutoConstants.RTSToNote5,
//                 AutoConstants.kMaxVelocity, AutoConstants.kMaxAcceleration, AutoConstants.kMaxAngularVelocity,
//                 AutoConstants.kMaxAngularAcceleration, 0.0));
//             break;
//           case '6':
//             runAutoCommand.andThen(drivebase.pathFindingThenFollowPath(AutoConstants.RBSToNote6,
//                 AutoConstants.kMaxVelocity, AutoConstants.kMaxAcceleration, AutoConstants.kMaxAngularVelocity,
//                 AutoConstants.kMaxAngularAcceleration, 0.0));
//             break;
//           case '7':
//             runAutoCommand.andThen(drivebase.pathFindingThenFollowPath(AutoConstants.RBSToNote7,
//                 AutoConstants.kMaxVelocity, AutoConstants.kMaxAcceleration, AutoConstants.kMaxAngularVelocity,
//                 AutoConstants.kMaxAngularAcceleration, 0.0));
//             break;
//           case '8':
//             runAutoCommand.andThen(drivebase.pathFindingThenFollowPath(AutoConstants.RBSToNote8,
//                 AutoConstants.kMaxVelocity, AutoConstants.kMaxAcceleration, AutoConstants.kMaxAngularVelocity,
//                 AutoConstants.kMaxAngularAcceleration, 0.0));
//             break;
//         }

//       } else if (pre == '4' || pre == '5' || pre == '6') {
//         switch (cur) {
//           case '1':
//             runAutoCommand.andThen(drivebase.followPathCommand(AutoConstants.RTSToNote1));
//             break;
//           case '2':
//             runAutoCommand.andThen(drivebase.followPathCommand(AutoConstants.RTSToNote2));
//             break;
//           case '3':
//             runAutoCommand.andThen(drivebase.followPathCommand(AutoConstants.RTSToNote3));
//             break;
//           case '4':
//             runAutoCommand.andThen(drivebase.followPathCommand(AutoConstants.RTSToNote4));
//             break;
//           case '5':
//             runAutoCommand.andThen(drivebase.followPathCommand(AutoConstants.RTSToNote5));
//             break;
//           case '6':
//             runAutoCommand.andThen(drivebase.followPathCommand(AutoConstants.RTSToNote6));
//             break;
//           case '7':
//             runAutoCommand.andThen(drivebase.followPathCommand(AutoConstants.RTSToNote7));
//             break;
//           case '8':
//             runAutoCommand.andThen(drivebase.followPathCommand(AutoConstants.RTSToNote8));
//             break;
//         }
//       } else if (pre == '7' || pre == '8') {
//         switch (cur) {
//           case '1':
//             runAutoCommand.andThen(drivebase.followPathCommand(AutoConstants.RBSToNote1));
//             break;
//           case '2':
//             runAutoCommand.andThen(drivebase.followPathCommand(AutoConstants.RBSToNote2));
//             break;
//           case '3':
//             runAutoCommand.andThen(drivebase.followPathCommand(AutoConstants.RBSToNote3));
//             break;
//           case '4':
//             runAutoCommand.andThen(drivebase.followPathCommand(AutoConstants.RBSToNote4));
//             break;
//           case '5':
//             runAutoCommand.andThen(drivebase.followPathCommand(AutoConstants.RBSToNote5));
//             break;
//           case '6':
//             runAutoCommand.andThen(drivebase.followPathCommand(AutoConstants.RBSToNote6));
//             break;
//           case '7':
//             runAutoCommand.andThen(drivebase.followPathCommand(AutoConstants.RBSToNote7));
//             break;
//           case '8':
//             runAutoCommand.andThen(drivebase.followPathCommand(AutoConstants.RBSToNote8));
//             break;
//         }
//       }

//       // whether rerun path or not
//       if (!drivebase.hasTargets()) {
//         if (firstTime) {
//           firstTime = false;
//           i--;
//           continue;
//         }
//         pre = cur;
//         firstTime = true;
//       }

//       // get note
//       runAutoCommand.andThen(new AutoIntakeCmd(drivebase, transportSubsystem, intakeSubsystem));

//       // move to shoot point and shoot
//       switch (cur) {
//         case '1':
//           runAutoCommand
//               .andThen(new AutoTransportShootCmd(drivebase, shooterSubsystem, transportSubsystem));
//           break;
//         case '2':
//           runAutoCommand
//               .andThen(new AutoTransportShootCmd(drivebase, shooterSubsystem, transportSubsystem));
//           break;
//         case '3':
//           runAutoCommand
//               .andThen(new AutoTransportShootCmd(drivebase, shooterSubsystem, transportSubsystem));
//           break;
//         case '4':
//           runAutoCommand.andThen(drivebase.pathFindingThenFollowPath(AutoConstants.topRelayToRTS,
//               AutoConstants.kMaxVelocity, AutoConstants.kMaxAcceleration, AutoConstants.kMaxAngularVelocity,
//               AutoConstants.kMaxAngularAcceleration, 0.0))
//               .andThen(new AutoTransportShootCmd(drivebase, shooterSubsystem, transportSubsystem));
//           break;
//         case '5':
//           runAutoCommand.andThen(drivebase.pathFindingThenFollowPath(AutoConstants.topRelayToRTS,
//               AutoConstants.kMaxVelocity, AutoConstants.kMaxAcceleration, AutoConstants.kMaxAngularVelocity,
//               AutoConstants.kMaxAngularAcceleration, 0.0))
//               .andThen(new AutoTransportShootCmd(drivebase, shooterSubsystem, transportSubsystem));
//           break;
//         case '6':
//           runAutoCommand.andThen(drivebase.pathFindingThenFollowPath(AutoConstants.topRelayToRTS,
//               AutoConstants.kMaxVelocity, AutoConstants.kMaxAcceleration, AutoConstants.kMaxAngularVelocity,
//               AutoConstants.kMaxAngularAcceleration, 0.0))
//               .andThen(new AutoTransportShootCmd(drivebase, shooterSubsystem, transportSubsystem));
//           break;
//         case '7':
//           runAutoCommand.andThen(drivebase.pathFindingThenFollowPath(AutoConstants.bottomRelayToRBS,
//               AutoConstants.kMaxVelocity, AutoConstants.kMaxAcceleration, AutoConstants.kMaxAngularVelocity,
//               AutoConstants.kMaxAngularAcceleration, 0.0))
//               .andThen(new AutoTransportShootCmd(drivebase, shooterSubsystem, transportSubsystem));
//           break;
//         case '8':
//           runAutoCommand.andThen(drivebase.pathFindingThenFollowPath(AutoConstants.bottomRelayToRBS,
//               AutoConstants.kMaxVelocity, AutoConstants.kMaxAcceleration, AutoConstants.kMaxAngularVelocity,
//               AutoConstants.kMaxAngularAcceleration, 0.0))
//               .andThen(new AutoTransportShootCmd(drivebase, shooterSubsystem, transportSubsystem));
//           break;
//       }
//     }

//     return new ParallelCommandGroup(runAutoCommand, runPeriodicCommand);
//   }

//   public static Command autoWithOnlyPose(Drivebase drivebase, RotateShooterSubsystem rotateShooterSubsystem,
//       ShooterSubsystem shooterSubsystem, TransportSubsystem transportSubsystem, IntakeSubsystem intakeSubsystem,
//       String autoNumber, String initial) {

//     int length = autoNumber.length();
//     Command runPeriodicCommand = new PoseRotateShooterCmd(rotateShooterSubsystem, drivebase);
//     Command runAutoCommand = new PoseShootCmd(drivebase, shooterSubsystem, transportSubsystem);
//     char pre = '0';
//     boolean firstTime = true;

//     // initial setting
//     switch (initial) {
//       case "left":
//         drivebase.resetPose(AutoConstants.leftPose2d);
//         break;
//       case "middle":
//         drivebase.resetPose(AutoConstants.middlePose2d);
//         break;
//       case "right":
//         drivebase.resetPose(AutoConstants.rightPose2d);
//         break;
//     }

//     // execute auto
//     for (int i = 0; i < length; i++) {
//       char cur = autoNumber.charAt(i);

//       // move to note
//       if (pre == '0' || pre == '1' || pre == '2' || pre == '3') {
//         switch (cur) {
//           case '1':
//             runAutoCommand.andThen(drivebase.pathFindingToPose(2.13, 6.68, 25.53, AutoConstants.kMaxVelocity,
//                 AutoConstants.kMaxAcceleration, AutoConstants.kMaxAngularVelocity,
//                 AutoConstants.kMaxAngularAcceleration, 0, 0));
//             break;
//           case '2':
//             runAutoCommand.andThen(drivebase.pathFindingToPose(2.13, 5.5, 0, AutoConstants.kMaxVelocity,
//                 AutoConstants.kMaxAcceleration, AutoConstants.kMaxAngularVelocity,
//                 AutoConstants.kMaxAngularAcceleration, 0, 0));
//             break;
//           case '3':
//             runAutoCommand.andThen(drivebase.pathFindingToPose(2.13, 4.4, -29.16, AutoConstants.kMaxVelocity,
//                 AutoConstants.kMaxAcceleration, AutoConstants.kMaxAngularVelocity,
//                 AutoConstants.kMaxAngularAcceleration, 0, 0));
//             break;
//           case '4':
//             runAutoCommand.andThen(drivebase.pathFindingThenFollowPath(AutoConstants.RTSToNote4,
//                 AutoConstants.kMaxVelocity, AutoConstants.kMaxAcceleration, AutoConstants.kMaxAngularVelocity,
//                 AutoConstants.kMaxAngularAcceleration, 0.0));
//             break;
//           case '5':
//             runAutoCommand.andThen(drivebase.pathFindingThenFollowPath(AutoConstants.RTSToNote5,
//                 AutoConstants.kMaxVelocity, AutoConstants.kMaxAcceleration, AutoConstants.kMaxAngularVelocity,
//                 AutoConstants.kMaxAngularAcceleration, 0.0));
//             break;
//           case '6':
//             runAutoCommand.andThen(drivebase.pathFindingThenFollowPath(AutoConstants.RBSToNote6,
//                 AutoConstants.kMaxVelocity, AutoConstants.kMaxAcceleration, AutoConstants.kMaxAngularVelocity,
//                 AutoConstants.kMaxAngularAcceleration, 0.0));
//             break;
//           case '7':
//             runAutoCommand.andThen(drivebase.pathFindingThenFollowPath(AutoConstants.RBSToNote7,
//                 AutoConstants.kMaxVelocity, AutoConstants.kMaxAcceleration, AutoConstants.kMaxAngularVelocity,
//                 AutoConstants.kMaxAngularAcceleration, 0.0));
//             break;
//           case '8':
//             runAutoCommand.andThen(drivebase.pathFindingThenFollowPath(AutoConstants.RBSToNote8,
//                 AutoConstants.kMaxVelocity, AutoConstants.kMaxAcceleration, AutoConstants.kMaxAngularVelocity,
//                 AutoConstants.kMaxAngularAcceleration, 0.0));
//             break;
//         }

//       } else if (pre == '4' || pre == '5' || pre == '6') {
//         switch (cur) {
//           case '1':
//             runAutoCommand.andThen(drivebase.followPathCommand(AutoConstants.RTSToNote1));
//             break;
//           case '2':
//             runAutoCommand.andThen(drivebase.followPathCommand(AutoConstants.RTSToNote2));
//             break;
//           case '3':
//             runAutoCommand.andThen(drivebase.followPathCommand(AutoConstants.RTSToNote3));
//             break;
//           case '4':
//             runAutoCommand.andThen(drivebase.followPathCommand(AutoConstants.RTSToNote4));
//             break;
//           case '5':
//             runAutoCommand.andThen(drivebase.followPathCommand(AutoConstants.RTSToNote5));
//             break;
//           case '6':
//             runAutoCommand.andThen(drivebase.followPathCommand(AutoConstants.RTSToNote6));
//             break;
//           case '7':
//             runAutoCommand.andThen(drivebase.followPathCommand(AutoConstants.RTSToNote7));
//             break;
//           case '8':
//             runAutoCommand.andThen(drivebase.followPathCommand(AutoConstants.RTSToNote8));
//             break;
//         }
//       } else if (pre == '7' || pre == '8') {
//         switch (cur) {
//           case '1':
//             runAutoCommand.andThen(drivebase.followPathCommand(AutoConstants.RBSToNote1));
//             break;
//           case '2':
//             runAutoCommand.andThen(drivebase.followPathCommand(AutoConstants.RBSToNote2));
//             break;
//           case '3':
//             runAutoCommand.andThen(drivebase.followPathCommand(AutoConstants.RBSToNote3));
//             break;
//           case '4':
//             runAutoCommand.andThen(drivebase.followPathCommand(AutoConstants.RBSToNote4));
//             break;
//           case '5':
//             runAutoCommand.andThen(drivebase.followPathCommand(AutoConstants.RBSToNote5));
//             break;
//           case '6':
//             runAutoCommand.andThen(drivebase.followPathCommand(AutoConstants.RBSToNote6));
//             break;
//           case '7':
//             runAutoCommand.andThen(drivebase.followPathCommand(AutoConstants.RBSToNote7));
//             break;
//           case '8':
//             runAutoCommand.andThen(drivebase.followPathCommand(AutoConstants.RBSToNote8));
//             break;
//         }
//       }

//       // get note
//       runAutoCommand.andThen(new AutoIntakeCmd(drivebase, transportSubsystem, intakeSubsystem));

//       // move to shoot point and shoot
//       switch (cur) {
//         case '1':
//           runAutoCommand
//               .andThen(new PoseShootCmd(drivebase, shooterSubsystem, transportSubsystem));
//           break;
//         case '2':
//           runAutoCommand
//               .andThen(new PoseShootCmd(drivebase, shooterSubsystem, transportSubsystem));
//           break;
//         case '3':
//           runAutoCommand
//               .andThen(new PoseShootCmd(drivebase, shooterSubsystem, transportSubsystem));
//           break;
//         case '4':
//           runAutoCommand.andThen(drivebase.pathFindingThenFollowPath(AutoConstants.topRelayToRTS,
//               AutoConstants.kMaxVelocity, AutoConstants.kMaxAcceleration, AutoConstants.kMaxAngularVelocity,
//               AutoConstants.kMaxAngularAcceleration, 0.0))
//               .andThen(new PoseShootCmd(drivebase, shooterSubsystem, transportSubsystem));
//           break;
//         case '5':
//           runAutoCommand.andThen(drivebase.pathFindingThenFollowPath(AutoConstants.topRelayToRTS,
//               AutoConstants.kMaxVelocity, AutoConstants.kMaxAcceleration, AutoConstants.kMaxAngularVelocity,
//               AutoConstants.kMaxAngularAcceleration, 0.0))
//               .andThen(new PoseShootCmd(drivebase, shooterSubsystem, transportSubsystem));
//           break;
//         case '6':
//           runAutoCommand.andThen(drivebase.pathFindingThenFollowPath(AutoConstants.topRelayToRTS,
//               AutoConstants.kMaxVelocity, AutoConstants.kMaxAcceleration, AutoConstants.kMaxAngularVelocity,
//               AutoConstants.kMaxAngularAcceleration, 0.0))
//               .andThen(new PoseShootCmd(drivebase, shooterSubsystem, transportSubsystem));
//           break;
//         case '7':
//           runAutoCommand.andThen(drivebase.pathFindingThenFollowPath(AutoConstants.bottomRelayToRBS,
//               AutoConstants.kMaxVelocity, AutoConstants.kMaxAcceleration, AutoConstants.kMaxAngularVelocity,
//               AutoConstants.kMaxAngularAcceleration, 0.0))
//               .andThen(new PoseShootCmd(drivebase, shooterSubsystem, transportSubsystem));
//           break;
//         case '8':
//           runAutoCommand.andThen(drivebase.pathFindingThenFollowPath(AutoConstants.bottomRelayToRBS,
//               AutoConstants.kMaxVelocity, AutoConstants.kMaxAcceleration, AutoConstants.kMaxAngularVelocity,
//               AutoConstants.kMaxAngularAcceleration, 0.0))
//               .andThen(new PoseShootCmd(drivebase, shooterSubsystem, transportSubsystem));
//           break;
//       }
//     }

//     return new ParallelCommandGroup(runPeriodicCommand, runAutoCommand);
//   }

//   private Autos() {
//     throw new UnsupportedOperationException("This is a utility class!");
//   }

// }
