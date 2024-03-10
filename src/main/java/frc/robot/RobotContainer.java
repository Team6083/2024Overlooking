// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Constants.DriveControllerConstants;
import frc.robot.commands.IntakeWithTransportCmd;
import frc.robot.commands.ReIntakeWithTransportCmd;
import frc.robot.commands.controllerCmds.SwerveJoystickCmd;
import frc.robot.commands.transportCmds.TransCmd;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.PowerDistributionSubsystem;
import frc.robot.subsystems.RotateShooterSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.TransportSubsystem;
import frc.robot.subsystems.drive.Drivebase;
import frc.robot.subsystems.visionProcessing.TagTracking;

public class RobotContainer {
  private final CommandXboxController mainController;
  private final PowerDistributionSubsystem powerDistributionSubsystem;
  private final Drivebase drivebase;
  private final IntakeSubsystem intakeSubsystem;
  private final ShooterSubsystem shooterSubsystem;
  private final TransportSubsystem transportSubsystem;
  private final RotateShooterSubsystem rotateShooterSubsystem;
  private final TagTracking tagTracking;

  public RobotContainer() {
    tagTracking = new TagTracking();
    // define subsystems
    mainController = new CommandXboxController(DriveControllerConstants.kMainController);
    powerDistributionSubsystem = new PowerDistributionSubsystem();
    drivebase = new Drivebase();
    rotateShooterSubsystem = new RotateShooterSubsystem(powerDistributionSubsystem, tagTracking);
    intakeSubsystem = new IntakeSubsystem(powerDistributionSubsystem);
    shooterSubsystem = new ShooterSubsystem(powerDistributionSubsystem);
    transportSubsystem = new TransportSubsystem(powerDistributionSubsystem);
    configureBindings();
  }

  private void configureBindings() {
    // drivetrain
    drivebase.setDefaultCommand(new SwerveJoystickCmd(drivebase, mainController));
    mainController.pov(0).onTrue(drivebase.accelerateCmd());
    mainController.pov(180).onTrue(drivebase.defaultSpeedCmd());

    // intake and transport
    mainController.y().toggleOnTrue(new IntakeWithTransportCmd(transportSubsystem, intakeSubsystem));
    mainController.x().whileTrue(new ReIntakeWithTransportCmd(transportSubsystem, intakeSubsystem));

    // shooter
    mainController.a().toggleOnTrue(new TransCmd(transportSubsystem, shooterSubsystem));

    mainController.b().toggleOnTrue(shooterSubsystem.shootPIDRateCmd());
    mainController.a().whileTrue(shooterSubsystem.setRateModeCmd(1).alongWith(rotateShooterSubsystem.setModeCmd(1)))
        .whileFalse(shooterSubsystem.setRateModeCmd(2).alongWith(rotateShooterSubsystem.setModeCmd(2)));
    mainController.leftBumper()
        .whileTrue(shooterSubsystem.setRateModeCmd(3).alongWith(rotateShooterSubsystem.setModeCmd(3)))
        .whileFalse(shooterSubsystem.setRateModeCmd(1).alongWith(rotateShooterSubsystem.setModeCmd(4)));
    // transport
    mainController.back().toggleOnTrue(transportSubsystem.setTransportCmd());
    // hook

    // semi-automatic

    // reset
  }

  public Command getAutonomousCommand() {
    return Commands.print("No autonomous command configured");
  }
}
