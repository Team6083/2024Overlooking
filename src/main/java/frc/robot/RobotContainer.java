// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandGenericHID;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Constants.DriveControllerConstants;
import frc.robot.commands.IntakeWithTransportCmd;
import frc.robot.commands.ReIntakeWithTransportCmd;
import frc.robot.commands.SwerveJoystickCmd;
import frc.robot.commands.TransportToShootCmd;
import frc.robot.subsystems.HookSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.PowerDistributionSubsystem;
import frc.robot.subsystems.RotateShooterSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.TransportSubsystem;
import frc.robot.subsystems.drive.Drivebase;
import frc.robot.subsystems.visionProcessing.TagTracking;

public class RobotContainer {
  private final CommandXboxController mainController;
  private final CommandGenericHID controlPanel;
  private final PowerDistributionSubsystem powerDistributionSubsystem;
  private final Drivebase drivebase;
  private final IntakeSubsystem intakeSubsystem;
  private final ShooterSubsystem shooterSubsystem;
  private final TransportSubsystem transportSubsystem;
  private final RotateShooterSubsystem rotateShooterSubsystem;
  private final HookSubsystem hookSubsystem;
  private final TagTracking tagTracking;

  public RobotContainer() {
    tagTracking = new TagTracking();
    // define subsystems
    mainController = new CommandXboxController(DriveControllerConstants.kMainController);
    controlPanel = new CommandGenericHID(DriveControllerConstants.kControlPanel);
    powerDistributionSubsystem = new PowerDistributionSubsystem();
    drivebase = new Drivebase();
    rotateShooterSubsystem = new RotateShooterSubsystem(powerDistributionSubsystem, tagTracking);
    intakeSubsystem = new IntakeSubsystem(powerDistributionSubsystem);
    shooterSubsystem = new ShooterSubsystem(powerDistributionSubsystem);
    transportSubsystem = new TransportSubsystem(powerDistributionSubsystem);
    hookSubsystem = new HookSubsystem(powerDistributionSubsystem);
    configureBindings();
  }

  private void configureBindings() {
    // drivetrain
    drivebase.setDefaultCommand(new SwerveJoystickCmd(drivebase, mainController));
    mainController.rightBumper().onTrue(drivebase.accelerateCmd());
    mainController.leftBumper().onTrue(drivebase.defaultSpeedCmd());

    // intake and transport
    mainController.y().toggleOnTrue(new IntakeWithTransportCmd(transportSubsystem, intakeSubsystem));
    mainController.x().whileTrue(new ReIntakeWithTransportCmd(transportSubsystem, intakeSubsystem));

    // shooter
    mainController.b().toggleOnTrue(shooterSubsystem.shootPIDRateCmd().alongWith(rotateShooterSubsystem.setModeCmd(1)))
        .toggleOnFalse(rotateShooterSubsystem.setModeCmd(4));
    controlPanel.button(0)
        .whileTrue(shooterSubsystem.setRateModeCmd(3).alongWith(rotateShooterSubsystem.setModeCmd(3)))
        .whileFalse(shooterSubsystem.setRateModeCmd(1).alongWith(rotateShooterSubsystem.setModeCmd(1)));
    controlPanel.button(1)
        .whileTrue(rotateShooterSubsystem.changeMaunalModeCmd(true))
        .whileFalse(rotateShooterSubsystem.changeMaunalModeCmd(false));

    // transport
    mainController.a().whileTrue(new TransportToShootCmd(transportSubsystem, shooterSubsystem));

    // hook
    mainController.rightTrigger(0.5).whileTrue(hookSubsystem.upAllCmd());
    mainController.leftTrigger(0.5).whileTrue(hookSubsystem.downAllCmd());
    controlPanel.button(4).whileTrue(hookSubsystem.leftUpIndivisual());
    controlPanel.button(5).whileTrue(hookSubsystem.leftDownIndivisual());
    controlPanel.button(6).whileTrue(hookSubsystem.rightUpIndivisual());
    controlPanel.button(7).whileTrue(hookSubsystem.rightDownIndivisual());

    // semi-automatic

    // reset
    mainController.back().onTrue(drivebase.gyroResetCmd());
  }

  public Command getAutonomousCommand() {
    return Commands.print("No autonomous command configured");
  }
}
