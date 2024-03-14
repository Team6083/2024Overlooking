// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.Map;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;

import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandGenericHID;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Constants.DriveControllerConstants;
import frc.robot.Constants.DrivebaseConstants;
import frc.robot.commands.IntakeWithTransportCmd;
import frc.robot.commands.ReIntakeWithTransportCmd;
import frc.robot.commands.TimeStopIntakeCmd;
import frc.robot.commands.autoCmds.AutoTransportToShootCmd;
import frc.robot.commands.driveControls.NoteDriveCmd;
import frc.robot.commands.driveControls.SwerveJoystickCmd;
import frc.robot.commands.driveControls.TagDriveCmd;
import frc.robot.subsystems.HookSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.PowerDistributionSubsystem;
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
    private final HookSubsystem hookSubsystem;
    private final TagTracking tagTracking;
    private final NoteTracking noteTracking;
    private SendableChooser<Command> autoChooser;
    // private SendableChooser<String> initialChooser;

    public RobotContainer() {
        tagTracking = new TagTracking();
        noteTracking = new NoteTracking();
        // define subsystems
        mainController = new CommandXboxController(DriveControllerConstants.kMainController);
        controlPanel = new CommandGenericHID(DriveControllerConstants.kControlPanel);
        powerDistributionSubsystem = new PowerDistributionSubsystem();
        drivebase = new Drivebase(tagTracking, noteTracking);
        intakeSubsystem = new IntakeSubsystem(powerDistributionSubsystem);
        shooterSubsystem = new ShooterSubsystem(tagTracking);
        transportSubsystem = new TransportSubsystem(powerDistributionSubsystem);
        hookSubsystem = new HookSubsystem(powerDistributionSubsystem);

        // NamedCommands.registerCommand("AutoTransportToShoot",
        // new AutoTransportToShootCmd(transportSubsystem, shooterSubsystem));
        // NamedCommands.registerCommand("AutoRotateShooter",
        // drivebase.tagTracking2Cmd());
        // NamedCommands.registerCommand("AutoTag",
        // new TagDriveCmd(drivebase, mainController));
        NamedCommands.registerCommand("AutoIntakeDown",
                new TimeStopIntakeCmd(intakeSubsystem).withTimeout(2.52));
        NamedCommands.registerCommand("AutoIntakeWithTransport",
                new IntakeWithTransportCmd(transportSubsystem, intakeSubsystem));
        NamedCommands.registerCommand("AutoAimControl",
                shooterSubsystem.aimControlCmd()); // rate and rotate
        NamedCommands.registerCommand("AutoTransport",
                transportSubsystem.transportIntakeCmd().withTimeout(0.5));
        NamedCommands.registerCommand("AutoNote",
                new NoteDriveCmd(drivebase, mainController).withTimeout(0.5));
        NamedCommands.registerCommand("AutoTag",
                new TagDriveCmd(drivebase, mainController));

        autoChooser = AutoBuilder.buildAutoChooser();

        SmartDashboard.putData("Auto Chooser", autoChooser);

        // initialChooser = new SendableChooser<String>();
        // initialChooser.setDefaultOption("none", "null");
        // initialChooser.addOption("left", "left");
        // initialChooser.addOption("middle", "middle");
        // initialChooser.addOption("right", "right");
        // SmartDashboard.putString("auto", "null");
        // SmartDashboard.putData(initialChooser);

        configureBindings();
    }

    private void configureBindings() {
        // drivetrain
        drivebase.setDefaultCommand(new SwerveJoystickCmd(drivebase, mainController));
        mainController.rightBumper()
                .onTrue(Commands.runOnce(() -> drivebase.setMagnification(DrivebaseConstants.kHighMagnification)));
        mainController.leftBumper()
                .onTrue(Commands.runOnce(() -> drivebase.setMagnification(DrivebaseConstants.kDefaultMagnification)));

        // intake and transport
        mainController.y()
                .toggleOnTrue(new IntakeWithTransportCmd(transportSubsystem, intakeSubsystem));
        mainController.x()
                .whileTrue(new ReIntakeWithTransportCmd(transportSubsystem, intakeSubsystem));
        controlPanel.button(5)
                .onTrue(intakeSubsystem.setUpIntakeCmd());
        controlPanel.button(6)
                .onTrue(intakeSubsystem.setDownIntakeCmd());

        // shooter
        enum ShooterModeSelector {
            Carry,
            AutoShoot,
        }

        shooterSubsystem
                .setDefaultCommand(shooterSubsystem.setInitRateControlCmd()
                        .alongWith(shooterSubsystem.manualOffsetAdjustCmd(controlPanel.getRawAxis(1))));
        mainController.b()
                .toggleOnTrue(Commands.select(
                        Map.ofEntries(
                                Map.entry(ShooterModeSelector.Carry, shooterSubsystem.carryControlCmd()),
                                Map.entry(ShooterModeSelector.AutoShoot,
                                        shooterSubsystem.aimControlCmd()
                                                .alongWith(new TagDriveCmd(drivebase, mainController)))),
                        () -> {
                            if (controlPanel.button(8).getAsBoolean()) {
                                return ShooterModeSelector.Carry;
                            }
                            if (controlPanel.button(9).getAsBoolean()) {
                                return ShooterModeSelector.AutoShoot;
                            }
                            return shooterSubsystem.fixRateControlCmd()
                                    .alongWith(new TagDriveCmd(drivebase, mainController));
                        }));

        mainController.pov(0).whileTrue(shooterSubsystem.manualUpCmd().onlyWhile(
                () -> shooterSubsystem.getIsManual()));
        mainController.pov(180).whileTrue(shooterSubsystem.manualDownCmd().onlyWhile(
                () -> shooterSubsystem.getIsManual()));
        controlPanel.button(10).onTrue(shooterSubsystem.isManualOnCmd()).onFalse(shooterSubsystem.isManualOffCmd());
        // tracking
        controlPanel.button(7)
                .whileTrue(new NoteDriveCmd(drivebase, mainController));
        mainController.b()
                .toggleOnTrue(new TagDriveCmd(drivebase, mainController));

        // transport
        mainController.a()
                .toggleOnTrue(
                        transportSubsystem.transportIntakeCmd().onlyWhile(
                                () -> shooterSubsystem.isEnoughRate()).withTimeout(0.5));
        mainController.start().toggleOnTrue(transportSubsystem.transportIntakeCmd().withTimeout(0.5));

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
        // if (autoChooser.getSelected().isScheduled()) {
        return autoChooser.getSelected();
        // }
        // String autoNumber = SmartDashboard.getString("auto", "null");
        // String initial = initialChooser.getSelected();
        // var alliance = DriverStation.getAlliance();
        // if (initial == "null" && alliance.isPresent())
        // return Commands.none();
        // boolean isRed = alliance.get() == DriverStation.Alliance.Red;
        // if (isRed) {
        // initial = (initial == "left" ? "right" : (initial == "right" ? "left" :
        // "middle"));
        // }
        // return Autos.autoOptimize(drivebase, shooterSubsystem, transportSubsystem,
        // intakeSubsystem, autoNumber, initial);
    }
}
