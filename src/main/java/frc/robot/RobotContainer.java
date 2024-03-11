// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandGenericHID;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Constants.DriveControllerConstants;
import frc.robot.Constants.RotateShooterConstants;
import frc.robot.commands.Autos;
import frc.robot.commands.IntakeWithTransportCmd;
import frc.robot.commands.ReIntakeWithTransportCmd;
import frc.robot.commands.ShooterDefaultCmd;
import frc.robot.commands.TransportToShootCmd;
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

    NamedCommands.registerCommand("AutoAim", rotateShooterSubsystem.setAutoAim());
    NamedCommands.registerCommand("AutoShootRate", shooterSubsystem.shootRateControlCmd());
    NamedCommands.registerCommand("AutoTransportToShoot",
        new TransportToShootCmd(transportSubsystem, shooterSubsystem));
    NamedCommands.registerCommand("AutoIntakeWithTransport",
        new IntakeWithTransportCmd(transportSubsystem, intakeSubsystem));
    NamedCommands.registerCommand("AutoFace", drivebase.tagTrackingCmd(0, 0, 0));
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
    mainController.b()
        .toggleOnTrue(shooterSubsystem.shootRateControlCmd().alongWith(rotateShooterSubsystem.setModeCmd(1))
            .alongWith(new TagDriveCmd(drivebase, mainController)))
        .toggleOnFalse(new ShooterDefaultCmd(shooterSubsystem, rotateShooterSubsystem));
    rotateShooterSubsystem.addErrorCmd(controlPanel.getRawAxis(0));///
    controlPanel.button(1)
        .whileTrue(shooterSubsystem.setRateModeCmd(2).alongWith(rotateShooterSubsystem.setModeCmd(2)))
        .whileFalse(new ShooterDefaultCmd(shooterSubsystem, rotateShooterSubsystem));
    controlPanel.button(2)///
        .whileTrue(rotateShooterSubsystem.changeMaunalModeCmd(true))
        .whileFalse(rotateShooterSubsystem.changeMaunalModeCmd(false));
    mainController.pov(0).whileTrue(rotateShooterSubsystem.setManualVoltageCmd(RotateShooterConstants.kManualVoltage));
    mainController.pov(180)
        .whileTrue(rotateShooterSubsystem.setManualVoltageCmd(-RotateShooterConstants.kManualVoltage));///
    // limelight
    controlPanel.button(3).whileTrue(drivebase.setTagVisionModeCmd());////
    // transport
    mainController.a().toggleOnTrue(new TransportToShootCmd(transportSubsystem, shooterSubsystem));

    // hook
    mainController.rightTrigger(0.5).whileTrue(hookSubsystem.upAllCmd());
    mainController.leftTrigger(0.5).whileTrue(hookSubsystem.downAllCmd());
    controlPanel.button(4).whileTrue(hookSubsystem.leftUpIndivisualCmd());
    controlPanel.button(5).whileTrue(hookSubsystem.leftDownIndivisualCmd());
    controlPanel.button(6).whileTrue(hookSubsystem.rightUpIndivisualCmd());
    controlPanel.button(7).whileTrue(hookSubsystem.rightDownIndivisualCmd());

    // reset
    mainController.back().onTrue(drivebase.gyroResetCmd());
  }

  public Command getAutonomousCommand() {
    if (autoChooser.getSelected().isScheduled()) {
      return autoChooser.getSelected();
    }
    String autoNumber = SmartDashboard.getString("auto", null);
    String initial = initialChooser.getSelected();
    var alliance = DriverStation.getAlliance();
    if (initial == null && alliance.isPresent())
      return Commands.none();
    boolean isRed = alliance.get() == DriverStation.Alliance.Red;
    if (isRed) {
      initial = (initial == "left" ? "right" : (initial == "right" ? "left" : "middle"));
    }
    return Autos.autoOptimize(drivebase, rotateShooterSubsystem, shooterSubsystem, transportSubsystem, intakeSubsystem,
        autoNumber, initial);
  }
}
