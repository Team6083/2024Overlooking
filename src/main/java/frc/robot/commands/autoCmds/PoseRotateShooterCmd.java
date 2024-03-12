// // Copyright (c) FIRST and other WPILib contributors.
// // Open Source Software; you can modify and/or share it under the terms of
// // the WPILib BSD license file in the root directory of this project.

// package frc.robot.commands.autoCmds;

// import edu.wpi.first.wpilibj2.command.Command;
// import frc.robot.subsystems.RotateShooterSubsystem;
// import frc.robot.subsystems.drive.Drivebase;

// public class PoseRotateShooterCmd extends Command {
//   private final RotateShooterSubsystem rotateShooterSubsystem;
//   private final Drivebase drivebase;

//   /** Creates a new AutoAimCmd. */
//   public PoseRotateShooterCmd(RotateShooterSubsystem rotateShooterSubsystem, Drivebase drivebase) {
//     this.rotateShooterSubsystem = rotateShooterSubsystem;
//     this.drivebase = drivebase;
//     addRequirements(this.rotateShooterSubsystem, this.drivebase);
//     // Use addRequirements() here to declare subsystem dependencies.
//   }

//   // Called when the command is initially scheduled.
//   @Override
//   public void initialize() {
//     rotateShooterSubsystem.setSetpoint(drivebase.calShooterAngleByPose2d());
//   }

//   // Called every time the scheduler runs while the command is scheduled.
//   @Override
//   public void execute() {
//     rotateShooterSubsystem.setSetpoint(drivebase.calShooterAngleByPose2d());
//   }

//   // Called once the command ends or is interrupted.
//   @Override
//   public void end(boolean interrupted) {
//   }

//   // Returns true when the command should end.
//   @Override
//   public boolean isFinished() {
//     return false;
//   }
// }
