// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;

import edu.wpi.first.networktables.NTSendable;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandGenericHID;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Constants.DriveControllerConstants;
// import frc.robot.commands.Autos;
import frc.robot.commands.IntakeWithTransportCmd;
import frc.robot.commands.ReIntakeWithTransportCmd;
import frc.robot.commands.TransportToShootCmd;
// import frc.robot.commands.autoCmds.AutoAimAndShootCmd;
import frc.robot.commands.autoCmds.AutoRotateShooterCmd;
// import frc.robot.commands.autoCmds.AutoTransportShootCmd;
import frc.robot.commands.driveControls.NoteDriveCmd;
import frc.robot.commands.driveControls.SwerveJoystickCmd;
import frc.robot.commands.driveControls.TagDriveCmd;
import frc.robot.subsystems.HookSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.PowerDistributionSubsystem;
import frc.robot.subsystems.RotateShooterSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.TransportSubsystem;
import frc.robot.subsystems.drive.Drivebase;
import frc.robot.subsystems.visionProcessing.NoteTracking;
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
  private final NoteTracking noteTracking;

  private SendableChooser<Command> autoChooser;
  private SendableChooser<String> initialChooser;

  public RobotContainer() {
    tagTracking = new TagTracking();
    noteTracking = new NoteTracking();
    // define subsystems
    mainController = new CommandXboxController(DriveControllerConstants.kMainController);
    controlPanel = new CommandGenericHID(DriveControllerConstants.kControlPanel);
    powerDistributionSubsystem = new PowerDistributionSubsystem();
    drivebase = new Drivebase(tagTracking, noteTracking);
    rotateShooterSubsystem = new RotateShooterSubsystem(powerDistributionSubsystem, tagTracking);
    intakeSubsystem = new IntakeSubsystem(powerDistributionSubsystem);
    shooterSubsystem = new ShooterSubsystem(powerDistributionSubsystem);
    transportSubsystem = new TransportSubsystem(powerDistributionSubsystem);
    hookSubsystem = new HookSubsystem(powerDistributionSubsystem);
    configureBindings();
    autoChooser = AutoBuilder.buildAutoChooser();

    SmartDashboard.putData("Auto Chooser", autoChooser);

    initialChooser = new SendableChooser<String>();
    initialChooser.setDefaultOption("none", null);
    initialChooser.addOption("left", "left");
    initialChooser.addOption("middle", "middle");
    initialChooser.addOption("right", "right");
    SmartDashboard.putString("auto", null);
    SmartDashboard.putData(initialChooser);

    NamedCommands.registerCommand("AutoAim", new AutoRotateShooterCmd(rotateShooterSubsystem));
    NamedCommands.registerCommand("AutoShootRate", shooterSubsystem.shootPIDRateCmd());
    // NamedCommands.registerCommand("AutoTransport", new AutoTransportShootCmd(drivebase, shooterSubsystem, transportSubsystem));
    NamedCommands.registerCommand("AutoIntakeWithTransport", new IntakeWithTransportCmd(transportSubsystem, intakeSubsystem));
    // NamedCommands.registerCommand("AutoFaceAndShoot",
    //     new AutoAimAndShootCmd(drivebase, rotateShooterSubsystem, shooterSubsystem, transportSubsystem));
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
    mainController.pov(0).whileTrue(rotateShooterSubsystem.addErrorCmd(1));
    mainController.pov(180).whileTrue(rotateShooterSubsystem.addErrorCmd(-1));
    controlPanel.button(1)
        .whileTrue(shooterSubsystem.setRateModeCmd(3).alongWith(rotateShooterSubsystem.setModeCmd(3)))
        .whileFalse(shooterSubsystem.setRateModeCmd(1).alongWith(rotateShooterSubsystem.setModeCmd(1)));
    controlPanel.button(2)
        .whileTrue(rotateShooterSubsystem.changeMaunalModeCmd(true))
        .whileFalse(rotateShooterSubsystem.changeMaunalModeCmd(false));

    // limelight
    controlPanel.button(3).whileTrue(new TagDriveCmd(drivebase, mainController));
    // transport
    mainController.a().toggleOnTrue(new TransportToShootCmd(transportSubsystem, shooterSubsystem));

    // hook
    mainController.rightTrigger(0.5).whileTrue(hookSubsystem.upAllCmd());
    mainController.leftTrigger(0.5).whileTrue(hookSubsystem.downAllCmd());
    controlPanel.button(4).whileTrue(hookSubsystem.leftUpIndivisual());
    controlPanel.button(5).whileTrue(hookSubsystem.leftDownIndivisual());
    controlPanel.button(6).whileTrue(hookSubsystem.rightUpIndivisual());
    controlPanel.button(7).whileTrue(hookSubsystem.rightDownIndivisual());

    // semi-automatic
    controlPanel.axisGreaterThan(0, 0).whileTrue(rotateShooterSubsystem.setAutoAim());

    // reset
    mainController.back().onTrue(drivebase.gyroResetCmd());
  }

  public Command getAutonomousCommand() {
    // String autoNumber = SmartDashboard.getString("auto", null);
    // String initial = initialChooser.getSelected();
    // var alliance = DriverStation.getAlliance();
    // if (initial == null && alliance.isPresent())
    //   return new InstantCommand();
    // Boolean isRed = alliance.get() == DriverStation.Alliance.Red;
    // if (isRed) {
    //   initial = (initial == "left" ? "right" : (initial == "right" ? "left" : "middle"));
    // }
    // // return Autos.auto(drivebase, riseShooter, shooter, transport, intake,
    // // autoNumber, initial);
    // return Autos.autoOptimize(drivebase, rotateShooterSubsystem, shooterSubsystem, transportSubsystem, intakeSubsystem,
    //     autoNumber, initial);
    // // return Commands.none();
    return Commands.print("No autonomous command configured");
  }
}
