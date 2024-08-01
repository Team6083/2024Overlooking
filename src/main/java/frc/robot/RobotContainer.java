// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.Map;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandGenericHID;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Constants.DriveControllerConstants;
import frc.robot.Constants.DrivebaseConstants;
import frc.robot.Constants.RotateShooterConstants;
import frc.robot.commands.IntakeWithTransportCmd;
import frc.robot.commands.ReIntakeWithTransportCmd;
import frc.robot.commands.driveControls.SwerveJoystickCmd;
import frc.robot.subsystems.HookSubsystem;
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
    private final HookSubsystem hookSubsystem;
    private final TagTracking tagTracking;

    public RobotContainer() {
        tagTracking = new TagTracking();
        mainController = new CommandXboxController(DriveControllerConstants.kMainController);
        controlPanel = new CommandGenericHID(DriveControllerConstants.kControlPanel);
        powerDistributionSubsystem = new PowerDistributionSubsystem();
        drivebase = new Drivebase(tagTracking);
        intakeSubsystem = new IntakeSubsystem(powerDistributionSubsystem);
        shooterSubsystem = new ShooterSubsystem(tagTracking);
        transportSubsystem = new TransportSubsystem(powerDistributionSubsystem);
        hookSubsystem = new HookSubsystem(powerDistributionSubsystem);

        SmartDashboard.putData("drivebase", drivebase);
        SmartDashboard.putData("shootSubsystem", shooterSubsystem);
        SmartDashboard.putData("IntakeSubsystem", intakeSubsystem);
        SmartDashboard.putData("HookSubsystem", hookSubsystem);
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
        mainController.y()
                .toggleOnTrue(new IntakeWithTransportCmd(transportSubsystem, intakeSubsystem));
        mainController.x()
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

        enum ShooterRotMode {
            Speaker,
            Amp,
            Carry,
            Manual
        }

        Map<ShooterRotMode, Command> shooterMap = Map.ofEntries(
                Map.entry(ShooterRotMode.Speaker,
                        shooterSubsystem
                                .speakerControlCmd(
                                        () -> controlPanel.getRawAxis(4) * RotateShooterConstants.kManualOffsetSupplierMulti,
                                        () -> controlPanel.button(12).getAsBoolean())
                                .alongWith(
                                        Commands.idle().until(() -> shooterSubsystem.canShoot(mainController.button(0).getAsBoolean()))
                                                .andThen(transportSubsystem.transportIntakeCmd()))
                                .withTimeout(3.0)),
                Map.entry(ShooterRotMode.Amp,
                        shooterSubsystem
                                .ampControlCmd(
                                        () -> controlPanel.button(12).getAsBoolean())
                                .alongWith(
                                        Commands.idle().until (() -> shooterSubsystem.canShoot(mainController.button(0).getAsBoolean()))
                                                .andThen(transportSubsystem.transportIntakeCmd()))
                                .withTimeout(4.5)),
                Map.entry(ShooterRotMode.Carry,
                        shooterSubsystem
                                .carryControlCmd(
                                        () -> controlPanel.button(12).getAsBoolean())
                                .alongWith(Commands.idle().until(() -> shooterSubsystem.canShoot(mainController.button(0).getAsBoolean()))
                                        .andThen(transportSubsystem.transportIntakeCmd()))
                                .withTimeout(2.0)));

        mainController.b()
                .toggleOnTrue(Commands.select(
                        shooterMap,
                        () -> {
                            if (controlPanel.button(11).getAsBoolean()) {
                                return ShooterRotMode.Carry;
                            }

                            if (controlPanel.button(10).getAsBoolean()) {
                                return ShooterRotMode.Amp;
                            }

                            return ShooterRotMode.Speaker;
                        }));
        shooterSubsystem.canShootButton(mainController.button(0));
        // hook
        mainController.rightTrigger(0.5)
                .whileTrue(hookSubsystem.upAllCmd());
        mainController.leftTrigger(0.5)
                .whileTrue(hookSubsystem.downAllCmd());
        controlPanel.button(1)
                .whileTrue(hookSubsystem.leftUpIndivisualCmd());
        controlPanel.button(2)
                .whileTrue(hookSubsystem.leftDownIndivisualCmd());
        controlPanel.button(3)
                .whileTrue(hookSubsystem.rightUpIndivisualCmd());
        controlPanel.button(4)
                .whileTrue(hookSubsystem.rightDownIndivisualCmd());

        // reset
        mainController.back().onTrue(drivebase.gyroResetCmd());
    }

    public Command getAutonomousCommand() {
        return null;
    }
}
