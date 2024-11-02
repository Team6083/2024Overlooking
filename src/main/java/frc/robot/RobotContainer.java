// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.lang.module.Configuration;
import java.nio.file.Path;
import java.util.List;
import java.util.Map;

import com.fasterxml.jackson.databind.introspect.WithMember;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;

import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.CommandGenericHID;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Constants.DriveControllerConstants;
import frc.robot.Constants.DrivebaseConstants;
import frc.robot.Constants.RotateShooterConstants;
// import frc.robot.commands.Autos;
import frc.robot.commands.IntakeWithTransportCmd;
import frc.robot.commands.ReIntakeWithTransportCmd;
import frc.robot.commands.TimeStopIntakeCmd;
import frc.robot.commands.TransportToShootCmd;
import frc.robot.commands.autoCmds.AutoTransportToShootCmd;
import frc.robot.commands.driveControls.SwerveJoystickCmd;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.PowerDistributionSubsystem;
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
    private final TagTracking tagTracking;
    private final SendableChooser<Command> autoChooser;

    public RobotContainer() {
        tagTracking = new TagTracking();
        mainController = new CommandXboxController(DriveControllerConstants.kMainController);
        controlPanel = new CommandGenericHID(DriveControllerConstants.kControlPanel);
        powerDistributionSubsystem = new PowerDistributionSubsystem();
        drivebase = new Drivebase(tagTracking);
        intakeSubsystem = new IntakeSubsystem(powerDistributionSubsystem);
        shooterSubsystem = new ShooterSubsystem(tagTracking);
        transportSubsystem = new TransportSubsystem(powerDistributionSubsystem);

        autoChooser = AutoBuilder.buildAutoChooser();
        
        NamedCommands.registerCommand("AutoIntakeDown",
                new TimeStopIntakeCmd(intakeSubsystem).withTimeout(2.52));
        NamedCommands.registerCommand("AutoIntakeWithTransport",
                new IntakeWithTransportCmd(transportSubsystem, intakeSubsystem));
        NamedCommands.registerCommand("AutoAimControl",
                shooterSubsystem.speakerControlCmd(() -> 2.0, () -> false)); // rate and rotate
        NamedCommands.registerCommand("AutoTransport",
                transportSubsystem.transportIntakeCmd().withTimeout(0.6));
        NamedCommands.registerCommand("AutoNote", new WaitCommand(0.01));
        NamedCommands.registerCommand("AutoTag",
                drivebase.tagTrackingCmd());

        
        autoChooser.setDefaultOption("Do Nothing", Commands.none());
        // autoChooser.addOption("Shoot", Autos.ShootCmd(drivebase, shooterSubsystem,
        // transportSubsystem,
        // mainController, intakeSubsystem));
        // autoChooser.addOption("ShootandForward", Autos.ShootandForwardCmd(drivebase,
        // transportSubsystem,
        // shooterSubsystem, mainController, intakeSubsystem ,this));
        SmartDashboard.putData("Auto Chooser", autoChooser);

        SmartDashboard.putData("drivebase", drivebase);
        SmartDashboard.putData("shootSubsystem", shooterSubsystem);
        SmartDashboard.putData("IntakeSubsystem", intakeSubsystem);
        SmartDashboard.putData("TransportSubsystem", transportSubsystem);
        SmartDashboard.putData("Drivebase", drivebase);

        configureBindings();

    }

    private void configureBindings() {
        // drivetrain
        drivebase.setDefaultCommand(new SwerveJoystickCmd(drivebase, mainController));
        mainController.rightBumper()
                .onTrue(Commands.runOnce(() -> drivebase
                        .setMagnification(DrivebaseConstants.kHighMagnification)));
        mainController.leftBumper()
                .onTrue(Commands.runOnce(() -> drivebase
                        .setMagnification(DrivebaseConstants.kDefaultMagnification)));

        // intake and transport
        mainController.axisGreaterThan(2, 0.5)
                .toggleOnTrue(new IntakeWithTransportCmd(transportSubsystem, intakeSubsystem));
        mainController.a()
                .whileTrue(new ReIntakeWithTransportCmd(transportSubsystem, intakeSubsystem));
        controlPanel.button(5)
                .whileTrue(intakeSubsystem.setUpIntakeCmd());
        controlPanel.button(6)
                .whileTrue(intakeSubsystem.setDownIntakeCmd());

        shooterSubsystem
                .setDefaultCommand(shooterSubsystem.initControlCmd());

        mainController.pov(0).whileTrue(
                shooterSubsystem.manualControlCmd(() -> 1)
                        .onlyWhile(() -> controlPanel.button(12).getAsBoolean()));
        mainController.pov(180).whileTrue(
                shooterSubsystem.manualControlCmd(() -> -1)
                        .onlyWhile(() -> controlPanel.button(12).getAsBoolean()));
        mainController.b().whileTrue(Commands.runEnd(transportSubsystem::setTransport, transportSubsystem::stopMotor));

        enum ShooterRotMode {
            Speaker,
            SpeakerAuto,
            Amp,
            Carry,
            Manual
        }

        Map<ShooterRotMode, Command> shooterMap = Map.ofEntries(
                Map.entry(ShooterRotMode.SpeakerAuto,
                        shooterSubsystem
                                .speakerControlCmd(
                                        () -> controlPanel.getRawAxis(3)
                                                * RotateShooterConstants.kManualOffsetSupplierMulti,
                                        () -> controlPanel.button(12)
                                                .getAsBoolean())
                                .alongWith(
                                        Commands.idle().until(
                                                () -> shooterSubsystem
                                                        .isEnoughRate())
                                                .andThen(transportSubsystem
                                                        .transportIntakeCmd()))
                                .withTimeout(5.0)),
                Map.entry(ShooterRotMode.Speaker,
                        shooterSubsystem
                                .speakerControlCmd(
                                        () -> controlPanel.getRawAxis(3)
                                                * RotateShooterConstants.kManualOffsetSupplierMulti,
                                        () -> controlPanel.button(12)
                                                .getAsBoolean())
                                .withTimeout(5.0)),
                Map.entry(ShooterRotMode.Amp,
                        shooterSubsystem
                                .ampControlCmd(
                                        () -> controlPanel.button(12)
                                                .getAsBoolean())
                                .alongWith(
                                        Commands.idle().until(
                                                () -> shooterSubsystem
                                                        .isEnoughRate())
                                                .andThen(transportSubsystem
                                                        .transportIntakeCmd()))
                                .withTimeout(5.0)),
                Map.entry(ShooterRotMode.Carry,
                        shooterSubsystem
                                .carryControlCmd(
                                        () -> controlPanel.button(12)
                                                .getAsBoolean())
                                .alongWith(Commands.idle().until(
                                        () -> shooterSubsystem.isEnoughRate())
                                        .andThen(transportSubsystem
                                                .transportIntakeCmd()))
                                .withTimeout(5.0)));

        mainController.axisGreaterThan(3, 0.5)
                .toggleOnTrue(Commands.select(
                        shooterMap,
                        () -> {
                            if (controlPanel.button(11).getAsBoolean()) {
                                return ShooterRotMode.Carry;
                            }

                            if (controlPanel.button(10).getAsBoolean()) {
                                return ShooterRotMode.Amp;
                            }
                            if (controlPanel.button(9).getAsBoolean()) {
                                return ShooterRotMode.SpeakerAuto;
                            }
                            return ShooterRotMode.Speaker;
                        }));

        // reset
        mainController.back().onTrue(drivebase.gyroResetCmd());
        controlPanel.button(7).onTrue(drivebase.gyroResetCmd());
    }

    public Command getAutonomousCommand() {
        return autoChooser.getSelected();
    }
}
