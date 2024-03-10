// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.drive;

import com.ctre.phoenix6.hardware.Pigeon2;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.path.PathPlannerPath;

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
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.DrivebaseConstants;
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

  private final PIDController facingNotePID;
  private final PIDController facingTagPID;
  private final PIDController followingTagPID;

  // face method value maybe correct
  private final double kP = 0.08;
  private final double kI = 0;
  private final double kD = 0;

  // fix distance value not determined yet
  private final double kfP = 0.8;
  private final double kfI = 0;
  private final double kfD = 0.006;

  // fix position
  public static final double kPP = 0.03;
  public static final double kII = 0;
  public static final double kDD = 0;

  private boolean noteTrackingCondition = false;
  private boolean tagTrackingCondition = false;

  // private final AHRS gyro;
  private final Pigeon2 gyro;

  private final Field2d field2d;

  private double magnification;

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

    facingNotePID = new PIDController(kP, kI, kD);
    followingTagPID = new PIDController(kfP, kfI, kfD);
    facingTagPID = new PIDController(kP, kI, kD);
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
    if (noteTrackingCondition) {
      rot = facingNoteRot(rot);
    }
    if (tagTrackingCondition) {
      rot = facingTag(rot);
    }
    swerveModuleStates = kinematics.toSwerveModuleStates(
        fieldRelative
            ? ChassisSpeeds.fromFieldRelativeSpeeds(xSpeed, ySpeed, rot, gyro.getRotation2d())
            : new ChassisSpeeds(xSpeed, ySpeed, rot));
    SwerveDriveKinematics.desaturateWheelSpeeds(swerveModuleStates, DrivebaseConstants.kMaxSpeed);
    frontLeft.setDesiredStateCmd(swerveModuleStates[0]);
    frontRight.setDesiredStateCmd(swerveModuleStates[1]);
    backLeft.setDesiredStateCmd(swerveModuleStates[2]);
    backRight.setDesiredStateCmd(swerveModuleStates[3]);
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

  public double facingNoteRot(double currentRot) {
    var target = noteTracking.getNotes();
    if (target.size() > 0) {
      var pose = target.get(0);
      double rot = -facingNotePID.calculate(pose.getX(), 0);
      return rot;
    } else {
      return currentRot;
    }
  }

  public double[] followingNoteSpeed() {
    var target = noteTracking.getNotes();
    double[] speed = new double[3];
    speed[0] = 0;
    speed[1] = 0;
    speed[2] = 0;
    if (target.size() > 0) {
      var pose = target.get(0);
      double XSpeed = facingNotePID.calculate(pose.getY(), NoteTrackingConstants.minNoteDistance);
      double YSpeed = 0;
      double rot = -facingNotePID.calculate(pose.getX(), 0);
      speed[0] = XSpeed;
      speed[1] = YSpeed;
      speed[2] = rot;
    }
    return speed;
  }

  public double facingTag(double currentRot) {
    double offset = tagTracking.getTx();
    double hasTarget = tagTracking.getTv();
    double targetID = tagTracking.getTID();
    if (hasTarget == 1 && targetID != 3.0 && targetID != 8.0) {
      double rot = -facingTagPID.calculate(offset, 0);
      return rot;
    }
    return currentRot;
  }

  /**
   * Return a double array. [0] xSpeed, [1] ySpeed, [2] rot
   * 
   * @return follow (double array)
   */
  public double[] followingTag() {
    double offset = tagTracking.getTx();
    double hasTarget = tagTracking.getTv();
    double[] speed = new double[3];
    double xSpeed = 0;
    double ySpeed = 0;
    double rot = 0;
    double x_dis = tagTracking.getBT()[2];
    if (hasTarget == 1) {
      rot = facingTagPID.calculate(offset, 0);
      xSpeed = -followingTagPID.calculate(x_dis, 0.5);
    }
    speed[0] = xSpeed;
    speed[1] = ySpeed;
    speed[2] = rot;
    return speed;
  }

  public void switchNoteTrackCondition() {
    noteTrackingCondition = !noteTrackingCondition;
  }

  public void switchTagTrackCondition() {
    tagTrackingCondition = !tagTrackingCondition;
  }

  public Command noteTrackCondition() {
    return Commands.runOnce(() -> switchNoteTrackCondition());
  }

  public Command tagTrackConditionCmd() {
    return Commands.runOnce(() -> switchTagTrackCondition());
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
    SmartDashboard.putNumber("frontLeft_speed", swerveModuleStates[0].speedMetersPerSecond);
    SmartDashboard.putNumber("frontRight_speed", swerveModuleStates[1].speedMetersPerSecond);
    SmartDashboard.putNumber("backLeft_speed", swerveModuleStates[2].speedMetersPerSecond);
    SmartDashboard.putNumber("backRight_speed", swerveModuleStates[3].speedMetersPerSecond);
    SmartDashboard.putNumber("gyro_heading", gyro.getRotation2d().getDegrees());
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
}
