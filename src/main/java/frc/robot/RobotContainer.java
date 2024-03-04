// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Constants.DriveControllerConstants;
import frc.robot.commands.controllerCmds.SwerveJoystickCmd;
import frc.robot.subsystems.drive.Drivebase;

public class RobotContainer {
  private final CommandXboxController mainController;
  private final Drivebase drivebase;

  public RobotContainer() {
    // define subsystems
    mainController = new CommandXboxController(DriveControllerConstants.kMainController);
    drivebase = new Drivebase();

    configureBindings();
  }

  private void configureBindings() {
    // buttons only
    drivebase.setDefaultCommand(new SwerveJoystickCmd(drivebase, mainController));
  }

  public Command getAutonomousCommand() {
    return Commands.print("No autonomous command configured");
  }
}
