package frc.robot;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;

public final class Constants {
    public static class FieldConstants {
        public static final double speakerFrontTall = Units.feetToMeters(6) + Units.inchesToMeters(11); // 210.82cm
        public static final double speakerBackTall = Units.feetToMeters(6) + Units.inchesToMeters(6); // 198.12cm
        public static final double speakerWidth = Units.feetToMeters(3) + Units.inchesToMeters(5.675); // 105.8545cm
        public static final double speakerExtend = Units.inchesToMeters(18); // 45.72cm
    }

    public static class DriveControllerConstants {
        public static final int kMainController = 0;
        public static final int kControlPanel = 1;
    }

    public static class ShooterConstants {
        public static final int kUpMotorChannel = 26;
        public static final int kDownMotorChannel = 25;
        public static final int kUpEncoderChannelA = 8;
        public static final int kUpEncoderChannelB = 9;
        public static final int kDownEncoderChannelA = 0;
        public static final int kDownEncoderChannelB = 1;
        public static final Boolean kUpMotorInverted = false;
        public static final Boolean kDownMotorInverted = false;
        public static final Boolean kUpEncoderInverted = true;
        public static final Boolean kDownEncoderInverted = true;
        public static final double kUpMotorManualVoltage = 10.0;
        public static final double kDownMotorManualVoltage = 10.0;
        public static final double[] kSpeakerShooterRate = { 60.0, 60.0 };
        public static final double[] kCarryShooterRate = { 30.0, 30.0 };
        public static final double[] kAmpShooterRate = { 39.0, 36.25 };
        public static final double kShooterRateOffset = 15;
        public static final double kP = 0.0;
        public static final double kI = 0.0;
        public static final double kD = 0.0;
        public static final double kUpMotorS = 2.37;
        public static final double kUpMotorV = 0.1238;
        public static final double kUpMotorA = 0.0;
        public static final double kDownMotorS = 0.681;
        public static final double kDownMotorV = 0.142;
        public static final double kDownMotorA = 0.0;
        public static final double kManualUpVoltage = 3;
        public static final double kManualDownVoltage = -3;
    }

    public static class RotateShooterConstants {
        public static final int kRotateShooterChannel = 21;
        public static final Boolean kRotateShooterInverted = true;
        public static final Boolean kEncoderInverted = true;
        public static final int kRotateEncoderChannel = 2;
        public static final double kInitDegree = 58.0;
        public static final double kCarryDegree = 20.0;
        public static final double kAmpDegree = 50.0;
        public static final double kManualVoltage = 4.0;
        public static final double kRotateVoltLimit = 5.0;
        public static final double kRotateAngleMin = 20.0;
        public static final double kRotateAngleMax = 67.0;
        public static final double kRotateDegreeErrorPoint = 3;
        public static final double kRotateOffset = 420.0;
        public static final double kShooterOffset = 4.32;
        public static final double kSpeakerHeight = 2.18;
        public static final double kAMPHeight = 0.89;
        public static final double kShooterHeight = 0.3;
        public static final double kP = 0.085;
        public static final double kI = 0.0002;
        public static final double kD = 0.0008;
    }

    public static class TransportConstants {
        public static final int kTransportChannel = 22;
        public static final boolean kTransportInverted = true;
        public static final double kTransVoltage = 4.5;
        public static final double kReTransVoltage = -4.5;
        public static final double kDistanceRange = 3.0;
    }

    public static class IntakeConstants {
        public static final int kIntakeChannel = 24;
        public static final int kRotateIntakeChannel = 29;
        public static final int kRotateEncoderChannel = 7;
        public static final Boolean kIntakeInverted = true;
        public static final Boolean kRotateIntakeInverted = false;
        public static final Boolean kRotateEncoderInverted = false;
        public static final double kRotateOffset = 105;
        public static final double kIntakeVoltage = 6.3;
        public static final double kThrowVoltage = -4.0;
        public static final double kRotateVoltage = 12.0;
        public static final double kStopTime = 2.52;
    }

    public static class HookConstants {
        public static final int kLineChannel = 23;
        public static final int kLeftChannel = 28;
        public static final int kRightChannel = 27;
        public static final boolean kLeftMotorInverted = true;
        public static final boolean kRightMotorInverted = false;
        public static final boolean kLineMotorInverted = false;
        public static final double kPHook = 2.0;
        public static final double kIHook = 0.0001;
        public static final double kDHook = 0.001;
        public static final double kPLine = 2.0;
        public static final double kILine = 0.0001;
        public static final double kDLine = 0.001;
        public static final int kLeftEncoderChannelA = 3;
        public static final int kLeftEncoderChannelB = 4;
        public static final int kRightEncoderChannelA = 5;
        public static final int kRightEncoderChannelB = 6;
        public static final double kHookPositionConversionfactor = 1.0;
        public static final double kLinePositionMax = 60.0;
        public static final double kLinePositionMin = 0.0;
        public static final double kHookPositionMax = 60.0;
        public static final double kHookPositionMin = 0.0;
        public static final double kLineVoltageLimit = 5.0;
        public static final double kLeftVoltageLimit = 5.0;
        public static final double kRightVoltageLimit = 5.0;
        public static final double kLeftSetpointModify = 3; // TO DO
        public static final double kRightSetpointModify = 3; // TO DO
        public static final double kLineSetpointModify = 3; // TO DO
        public static final double kOffsetLimit = 20; // TO DO
    }

    public static class DrivebaseConstants {
        // drive motor channel
        public static final int kFrontLeftDriveMotorChannel = 11;
        public static final int kFrontRightDriveMotorChannel = 15;
        public static final int kBackLeftDriveMotorChannel = 13;
        public static final int kBackRightDriveMotorChannel = 17;

        // turning motor channel
        public static final int kFrontLeftTurningMotorChannel = 12;
        public static final int kFrontRightTurningMotorChannel = 16;
        public static final int kBackLeftTurningMotorChannel = 14;
        public static final int kBackRightTurningMotorChannel = 18;

        // turnning encoder channel
        public static final int kFrontLeftTurningEncoderChannel = 31;
        public static final int kFrontRightTurningEncoderChannel = 32;
        public static final int kBackLeftTurningEncoderChannel = 33;
        public static final int kBackRightTurningEncoderChannel = 34;

        // can coder magnet offset value
        public static final double kFrontLeftCanCoderMagOffset = 0.003174;
        public static final double kFrontRightCanCoderMagOffset = 0.042236 ;
        public static final double kBackLeftCanCoderMagOffset = 0.347656;
        public static final double kBackRightCanCoderMagOffset = -0.178955;

        public static final double kMaxSpeed = 5;
        public static final double kMinRot = 0.1;
        public static final double kMaxAngularSpeed = 2.5 * Math.PI; // 1/2 rotation per second

        public static final double kXLimiterRateLimit = 5.0;
        public static final double kYLimiterRateLimit = 5.0;
        public static final double kRotLimiterRateLimit = 5.0;

        public static final boolean kFrontLeftDriveMotorInverted = false;
        public static final boolean kFrontRightDriveMotorInverted = true;
        public static final boolean kBackLeftDriveMotorInverted = false;
        public static final boolean kBackRightDriveMotorInverted = true;

        public static final boolean kGyroInverted = false; // wheather gyro is under the robot
        public static final double kGyroOffSet = 0;

        public static final double kDefaultMagnification = 0.8;
        public static final double kHighMagnification = 1.0;

        public static final double kTrackingP = 0.08;
        public static final double kTrackingI = 0.0;
        public static final double kTrackingD = 0.0;
    }

    public static final class ModuleConstants {
        public static final double kWheelRadius = 0.046;

        public static final double kWheelDiameterMeters = 0.15;

        public static final double kMaxModuleDriveVoltage = 4.0;

        public static final double kDriveClosedLoopRampRate = 0.1;// 1 second 1 unit
        public static final double kTurningClosedLoopRampRate = 0.1;

        public static final double kDesireSpeedtoMotorVoltage = kMaxModuleDriveVoltage / DrivebaseConstants.kMaxSpeed;

        public static final double kMaxModuleTuringVoltage = 10.0/3.0f;

        public static final double kMaxSpeedTurningDegree = 160.0;

        public static final double kPRotationController = kMaxModuleTuringVoltage / kMaxSpeedTurningDegree;
        public static final double kIRotationController = 0.0;
        public static final double kDRotationController = 0.0004;

        public static final boolean kTurningMotorInverted = true;
    }

    public static final class PowerDistributionConstants {
        public static final int kIntakeMotorCurrrentchannel = 2;
        public static final int kShooterUpMotorCurrentchannel = 13;
        public static final int kShooterDownMotorCurrentchannel = 17;
        public static final int kLineCurrentchannel = 6;
        public static final int kHookLeftMotorCurrentchannel = 5;
        public static final int kHookRightMotorCurrentchannel = 3;
        public static final int kTransportCurrentchannel = 16;
        public static final int kRiseShooterCurrentchannel = 4;
        public static final int kRotateIntakeCurrentchannel = 14;

        public static final double kIntakeMotorMaxCurrent = 40.0;
        public static final double kShooterUpMotorMaxCurrent = 40.0;
        public static final double kShooterDownMotorMaxCuurent = 40.0;
        public static final double kLineMotorMaxCurrent = 40.0;
        public static final double kHookLeftMotorMaxCurrent = 40.0;
        public static final double kHookRightaxCurrent = 40.0;
        public static final double kTransportMaxCurrent = 30.0;
        public static final double kRotateShooterMaxCurrent = 40.0;
        public static final double kRotateIntakeMaxCurrent = 30.0;
    }

    public static final class AutoConstants {
        public static final double kPTranslation = 4.0;
        public static final double kITranslation = 0.001;
        public static final double kDTranslation = 0.01;
        public static final double kPRotation = 2.0;
        public static final double kIRotation = 0.001;
        public static final double kDRotation = 0.01;
        public static final double kMaxModuleSpeed = 5.0;
        public static final double kDrivebaseRadius = 0.282575;

        public static final double kMaxVelocity = 3.35;
        public static final double kMaxAcceleration = 4;
        public static final double kMaxAngularVelocity = 453.38;
        public static final double kMaxAngularAcceleration = 240;
        public static final double kRotationDelayDistance = 0.0;

        public static final Pose2d leftPose2d = new Pose2d(0.76, 6.53, Rotation2d.fromDegrees(60));
        public static final Pose2d middlePose2d = new Pose2d(1.24, 5.5, Rotation2d.fromDegrees(0));
        public static final Pose2d rightPose2d = new Pose2d(0.76, 4.56, Rotation2d.fromDegrees(-60));
    }

    public static final class NoteTrackingConstants {
        public static final String kCameraName = "Microsoft_LifeCam_HD-3000";
        public static final int noteTrakingPipeline = 1;
        public static final double cameraHeight = 0.36;
        public static final double cameraWeight = 0.0;
        public static final double kCamYOffset = 0.1; // not sure yet
        public static final double pitchDegree = -30.0;
        public static final double yawDegree = 0;
        public static final double minNoteDistance = 0.2;
    }

    public static final class TagTrackingConstants {
        public static final double shooterHeight = 0.31;
        public static final double camHeight = 0.615;
        public static final double camPitch = 10.0;
        public static final double camToShooterDistance = 0.11;
    }
}
