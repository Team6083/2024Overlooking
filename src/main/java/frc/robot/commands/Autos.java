// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;


import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;

import frc.robot.Constants.AutoConstants;

// import frc.robot.commands.autoCmds.PoseRotateShooterCmd;

// import frc.robot.commands.driveControls.NoteDriveCmd;

import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.drive.Drivebase;

public final class Autos {



public Command ShootCmd(Drivebase drivebase, ShooterSubsystem shooterSubsystem) {
        Command AutoAimControl = shooterSubsystem.speakerControlCmd(null, null);

        drivebase.resetPose(AutoConstants.middlePose2d);

        Command cmd = new ParallelCommandGroup(AutoAimControl);

        return cmd;
    }
    
         private Autos() {
        throw new UnsupportedOperationException("This is a utility class!");
    }

}