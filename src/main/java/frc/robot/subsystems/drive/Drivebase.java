// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.drive;

import com.ctre.phoenix6.hardware.Pigeon2;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.commands.FollowPathHolonomic;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.util.HolonomicPathFollowerConfig;
import com.pathplanner.lib.util.PIDConstants;
import com.pathplanner.lib.util.ReplanningConfig;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.DrivebaseConstants;
import frc.robot.Constants.FieldConstants;
import frc.robot.Constants.NoteTrackingConstants;
import frc.robot.subsystems.visionProcessing.NoteTracking;
import frc.robot.subsystems.visionProcessing.TagTracking;

public class Drivebase extends SubsystemBase {
  /** Creates a new Drivetain. */
  private final Translation2d frontLeftLocation;
  private final Translation2d frontRightLocation;
  private final Translation2d backLeftLocation;
  private final Translation2d backRightLocation;

  private final SwerveModule frontLeft;
  private final SwerveModule frontRight;
  private final SwerveModule backLeft;
  private final SwerveModule backRight;

  private final SwerveDriveKinematics kinematics;
  private final SwerveDriveOdometry odometry;

  private final TagTracking tagTracking;
  private final NoteTracking noteTracking;

  private final PIDController trackingPID;

  // private final AHRS gyro;
  private final Pigeon2 gyro;

  private final Field2d field2d;

  private double magnification;

  private boolean isTagTracking = false;

  private SwerveModuleState[] swerveModuleStates = new SwerveModuleState[4];

  public Drivebase(TagTracking tagTracking, NoteTracking noteTracking) {
    this.tagTracking = tagTracking;
    this.noteTracking = noteTracking;
    frontLeftLocation = new Translation2d(0.3, 0.3);
    frontRightLocation = new Translation2d(0.3, -0.3);
    backLeftLocation = new Translation2d(-0.3, 0.3);
    backRightLocation = new Translation2d(-0.3, -0.3);

    frontLeft = new SwerveModule(DrivebaseConstants.kFrontLeftDriveMotorChannel,
        DrivebaseConstants.kFrontLeftTurningMotorChannel, DrivebaseConstants.kFrontLeftTurningEncoderChannel,
        DrivebaseConstants.kFrontLeftDriveMotorInverted, DrivebaseConstants.kFrontLeftCanCoderMagOffset, "frontLeft");
    frontRight = new SwerveModule(DrivebaseConstants.kFrontRightDriveMotorChannel,
        DrivebaseConstants.kFrontRightTurningMotorChannel, DrivebaseConstants.kFrontRightTurningEncoderChannel,
        DrivebaseConstants.kFrontRightDriveMotorInverted, DrivebaseConstants.kFrontRightCanCoderMagOffset,
        "frontRight");
    backLeft = new SwerveModule(DrivebaseConstants.kBackLeftDriveMotorChannel,
        DrivebaseConstants.kBackLeftTurningMotorChannel, DrivebaseConstants.kBackLeftTurningEncoderChannel,
        DrivebaseConstants.kBackLeftDriveMotorInverted, DrivebaseConstants.kBackLeftCanCoderMagOffset, "backLeft");
    backRight = new SwerveModule(DrivebaseConstants.kBackRightDriveMotorChannel,
        DrivebaseConstants.kBackRightTurningMotorChannel, DrivebaseConstants.kBackRightTurningEncoderChannel,
        DrivebaseConstants.kBackRightDriveMotorInverted, DrivebaseConstants.kBackRightCanCoderMagOffset, "backRight");

    SmartDashboard.putData("frontLeft", frontLeft);
    SmartDashboard.putData("frontRight", frontRight);
    SmartDashboard.putData("backLeft", backLeft);
    SmartDashboard.putData("backRight", backRight);

    // gyro = new AHRS(Port.kMXP);
    gyro = new Pigeon2(30);

    kinematics = new SwerveDriveKinematics(
        frontLeftLocation, frontRightLocation, backLeftLocation, backRightLocation);

    // create the odometry
    odometry = new SwerveDriveOdometry(
        kinematics,
        gyro.getRotation2d(),
        new SwerveModulePosition[] {
            frontLeft.getPosition(),
            frontRight.getPosition(),
            backLeft.getPosition(),
            backRight.getPosition()
        });
    field2d = new Field2d();

    // initialize magnification value
    magnification = 1.0;

    // reset the gyro
    resetGyro();

    // set the swerve speed equal 0
    drive(0, 0, 0, false);

    trackingPID = new PIDController(DrivebaseConstants.kTrackingP, DrivebaseConstants.kTrackingI,
        DrivebaseConstants.kTrackingD);
    AutoBuilder.configureHolonomic(
        this::getPose2d, // Robot pose suppier
        this::resetPose, // Method to reset odometry (will be called if your auto has a starting pose)
        this::getRobotRelativeSpeeds, // ChassisSpeeds supplier. MUST BE ROBOT RELATIVE
        this::driveRobotRelative, // Method that will drive the robot given ROBOT RELATIVE ChassisSpeeds
        new HolonomicPathFollowerConfig( // HolonomicPathFollowerConfig, this should likely live in your Constants class
            new PIDConstants(AutoConstants.kPTranslation, AutoConstants.kITranslation, AutoConstants.kDTranslation), // Translation
                                                                                                                     // PID
                                                                                                                     // constants
            new PIDConstants(AutoConstants.kPRotation, AutoConstants.kIRotation, AutoConstants.kDRotation), // Rotation
                                                                                                            // PID
                                                                                                            // constants
            DrivebaseConstants.kMaxSpeed, // Max module speed, in m/s
            AutoConstants.kDrivebaseRadius, // Drive base radius in meters. Distance from robot center to furthest
                                            // module.
            new ReplanningConfig() // Default path replanning config. See the API for the options here
        ),
        () -> {
          // Boolean supplier that controls when the path will be mirrored for the red
          // alliance
          // This will flip the path being followed to the red side of the field.
          // THE ORIGIN WILL REMAIN ON THE BLUE SIDE
          var alliance = DriverStation.getAlliance();
          if (alliance.isPresent()) {
            return alliance.get() == DriverStation.Alliance.Red;
          }
          return false;
        },
        this // Reference to this subsystem to set requirements
    );
  }

  public void resetGyro() {
    gyro.reset();
  }

  public void resetPose(Pose2d pose) {
    odometry.resetPosition(
        gyro.getRotation2d(),
        new SwerveModulePosition[] {
            frontLeft.getPosition(),
            frontRight.getPosition(),
            backLeft.getPosition(),
            backRight.getPosition()
        },
        pose);
  }

  public Rotation2d getRotation2dDegrees() {
    return Rotation2d.fromDegrees(DrivebaseConstants.kGyroOffSet
        + ((DrivebaseConstants.kGyroInverted) ? (360.0 - gyro.getRotation2d().getDegrees())
            : gyro.getRotation2d().getDegrees()));
  }

  /**
   * Method to drive the robot using joystick info.
   *
   * @param ySpeed        Speed of the robot in the y direction (forward).
   * @param xSpeed        Speed of the robot in the x direction (sideways).
   * @param rot           Angular rate of the robot.
   * @param fieldRelative Whether the provided x and y speeds are relative to the
   *                      field.
   * 
   *                      using the wpi function to set the speed of the swerve
   */
  public void drive(double xSpeed, double ySpeed, double rot, boolean fieldRelative) {

    swerveModuleStates = kinematics.toSwerveModuleStates(
        fieldRelative
            ? ChassisSpeeds.fromFieldRelativeSpeeds(xSpeed, ySpeed, rot, gyro.getRotation2d())
            : new ChassisSpeeds(xSpeed, ySpeed, rot));
    SwerveDriveKinematics.desaturateWheelSpeeds(swerveModuleStates, DrivebaseConstants.kMaxSpeed);
    frontLeft.setDesiredState(swerveModuleStates[0]);
    frontRight.setDesiredState(swerveModuleStates[1]);
    backLeft.setDesiredState(swerveModuleStates[2]);
    backRight.setDesiredState(swerveModuleStates[3]);
  }

  public void driveRobotRelative(ChassisSpeeds speeds) {
    drive(speeds.vxMetersPerSecond, speeds.vyMetersPerSecond, speeds.omegaRadiansPerSecond, false);
  }

  public ChassisSpeeds getRobotRelativeSpeeds() {
    return kinematics.toChassisSpeeds(
        frontLeft.getState(),
        frontRight.getState(),
        backLeft.getState(),
        backRight.getState());
  }

  public boolean hasTargets() {
    return noteTracking.hasTargets();
  }

  public double getFacingTagRot(double currentRot) {
    if (tagTracking.getTv() == 1 && tagTracking.getTID() != 3.0 && tagTracking.getTID() != 8.0) {
      return -trackingPID.calculate(tagTracking.getTx(), 0.0);
    }
    return currentRot;
  }

  public double getFacingNoteRot(double currentRot) {
    if (noteTracking.getTx().size() != 0) {
      double yaw = noteTracking.getTx().get(0);
      return -trackingPID.calculate(yaw, 0);
    }
    return currentRot;
  }

  public void tagTracking(double xSpeed, double ySpeed, double rot) {
    double robotRot = getFacingTagRot(rot);
    if (Math.abs(rot) > DrivebaseConstants.kMinRot) {
      robotRot = rot;
    }
    drive(xSpeed, ySpeed, robotRot, true);
  }

  public void noteTracking(double xSpeed, double ySpeed, double rot) {
    double robotRot = rot;
    if (noteTracking.getTx().size() != 0) {
      double yaw = noteTracking.getTx().indexOf(0.0);
      robotRot = -trackingPID.calculate(yaw, 0);
    }
    if (Math.abs(rot) > DrivebaseConstants.kMinRot) {
      robotRot = rot;
    }
    drive(xSpeed, ySpeed, robotRot, true);
  }

  public void tagTracking2() {
    double Rot = 0;
    double xSpeed = 0;
    double ySpeed = 0;
    double offset = tagTracking.getTx();
    if (tagTracking.getTv() == 1) {

      Rot = -trackingPID.calculate(offset, 0);
    }
    if (Math.abs(offset) >= 0.05) {
      isTagTracking=true;
      drive(xSpeed, ySpeed, Rot, true);
    } else{
      isTagTracking = false;
    }
  }

  public Command tagTrackingCmd(double xSpeed, double ySpeed, double rot) {
    Command cmd = runEnd(
        () -> tagTracking(xSpeed, ySpeed, rot),
        () -> drive(0, 0, 0, false));
    cmd.setName("TagTrackingCmd");
    return cmd;
  }

  public Command accelerateCmd() {
    Command cmd = Commands.runOnce(
        () -> setMagnification(DrivebaseConstants.kHighMagnification),
        this);
    cmd.setName("accelerateCmd");
    return cmd;
  }

  public Command defaultSpeedCmd() {
    Command cmd = Commands.runOnce(
        () -> setMagnification(DrivebaseConstants.kDefaultMagnification),
        this);
    cmd.setName("defaultSpeedCmd");
    return cmd;
  }

  public void setMagnification(double magnification) {
    this.magnification = magnification;
  }

  public double getMagnification() {
    return magnification;
  }

  /** Updates the field relative position of the robot. */
  public void updateOdometry() {
    odometry.update(
        gyro.getRotation2d(),
        new SwerveModulePosition[] {
            frontLeft.getPosition(),
            frontRight.getPosition(),
            backLeft.getPosition(),
            backRight.getPosition()
        });
  }

  public Pose2d getPose2d() {
    return odometry.getPoseMeters();
  }

  public void resetPose2dAndEncoder() {
    frontLeft.resetAllEncoder();
    frontRight.resetAllEncoder();
    backLeft.resetAllEncoder();
    backRight.resetAllEncoder();
    resetPose(new Pose2d(0, 0, new Rotation2d(0)));
  }

  public void putDashboard() {
    // SmartDashboard.putNumber("frontLeft_speed",
    // swerveModuleStates[0].speedMetersPerSecond);
    // SmartDashboard.putNumber("frontRight_speed",
    // swerveModuleStates[1].speedMetersPerSecond);
    // SmartDashboard.putNumber("backLeft_speed",
    // swerveModuleStates[2].speedMetersPerSecond);
    // SmartDashboard.putNumber("backRight_speed",
    // swerveModuleStates[3].speedMetersPerSecond);
    SmartDashboard.putNumber("gyro_heading", gyro.getRotation2d().getDegrees());
  }

  public double calShooterAngleByPose2d() {
    double x = getPose2d().getX();
    double y = getPose2d().getY();
    double distance = Math.sqrt(x * x + y * y);
    double backAngleDegree = Math.toDegrees(Math.atan((FieldConstants.speakerBackTall - 32.464) / distance));
    double frontAngleDegree = Math.toDegrees(Math.atan((FieldConstants.speakerFrontTall - 32.464) / distance));
    return (backAngleDegree + frontAngleDegree) / 2;
  }

  // auto drive
  public Command followPathCommand(String pathName) {
    PathPlannerPath path = PathPlannerPath.fromPathFile(pathName);

    return new FollowPathHolonomic(
        path,
        this::getPose2d, // Robot pose supplier
        this::getRobotRelativeSpeeds, // ChassisSpeeds supplier. MUST BE ROBOT RELATIVE
        this::driveRobotRelative, // Method that will drive the robot given ROBOT RELATIVE ChassisSpeeds
        new HolonomicPathFollowerConfig( // HolonomicPathFollowerConfig, this should likely live in your Constants class
            new PIDConstants(AutoConstants.kPTranslation, AutoConstants.kITranslation, AutoConstants.kDTranslation), // Translation
                                                                                                                     // PID
                                                                                                                     // constants
            new PIDConstants(AutoConstants.kPRotation, AutoConstants.kIRotation, AutoConstants.kDRotation), // Rotation
                                                                                                            // PID
                                                                                                            // constants
            DrivebaseConstants.kMaxSpeed, // Max module speed, in m/s
            AutoConstants.kDrivebaseRadius, // Drive base radius in meters. Distance from robot center to furthest
                                            // module.
            new ReplanningConfig() // Default path replanning config. See the API for the options here
        ),
        () -> {
          // Boolean supplier that controls when the path will be mirrored for the red
          // alliance
          // This will flip the path being followed to the red side of the field.
          // THE ORIGIN WILL REMAIN ON THE BLUE SIDE

          var alliance = DriverStation.getAlliance();
          if (alliance.isPresent()) {
            return alliance.get() == DriverStation.Alliance.Red;
          }
          return false;
        },
        this // Reference to this subsystem to set requirementsme
    );
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    updateOdometry();
    field2d.setRobotPose(getPose2d());
    putDashboard();
  }

  public Command gyroResetCmd() {
    Command cmd = this.runOnce(this::resetGyro);
    cmd.setName("gyroResetCmd");
    return cmd;
  }

  public Command pathFindingThenFollowPath(String pathName, double maxVelocity, double maxAcceleration,
      double maxAngularVelocity, double maxAngularAcceleration, double rotationDelayDistance) {
    PathPlannerPath path = PathPlannerPath.fromPathFile(pathName);
    PathConstraints constraints = new PathConstraints(
        maxVelocity, maxAcceleration,
        Units.degreesToRadians(maxAngularVelocity), Units.degreesToRadians(maxAngularAcceleration));
    return AutoBuilder.pathfindThenFollowPath(path, constraints, rotationDelayDistance);
  }

  public Command pathFindingToPose(double x, double y, double degrees, double maxVelocity, double maxAcceleration,
      double maxAngularVelocity, double maxAngularAcceleration, double goalEndVelocity,
      double rotationDelayDistance) {

    Pose2d targetPose = new Pose2d(x, y, Rotation2d.fromDegrees(degrees));
    PathConstraints constraints = new PathConstraints(
        maxVelocity, maxAcceleration,
        Units.degreesToRadians(maxAngularVelocity), Units.degreesToRadians(maxAngularAcceleration));
    return AutoBuilder.pathfindToPose(targetPose, constraints, goalEndVelocity, rotationDelayDistance);
  }

  public Command setTagVisionModeCmd() {
    Command cmd = Commands.runEnd(() -> tagTracking.isVisionOn(), () -> tagTracking.setCamMode());
    return cmd;
  }

  public Command tagTracking2Cmd() {
    Command cmd = Commands.runEnd(() -> tagTracking.setAutoCamOn(), () -> tagTracking.setAutoCamOff()).onlyWhile(()->isTagTracking);
    cmd.setName("tagTrackin2Cmd");
    return cmd;
  }

}