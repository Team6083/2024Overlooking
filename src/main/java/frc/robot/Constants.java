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
        public static final int kControlPanel = 2;
    }

    public static class ShooterConstants {
        public static final int kUpMotorChannel = 25;
        public static final int kDownMotorChannel = 26;
        public static final int kUpEncoderChannelA = 0;
        public static final int kUpEncoderChannelB = 1;
        public static final int kDownEncoderChannelA = 8;
        public static final int kDownEncoderChannelB = 9;
        public static final Boolean kUpMotorInverted = false;
        public static final Boolean kDownMotorInverted = false;
        public static final Boolean kUpEncoderInverted = true;
        public static final Boolean kDownEncoderInverted = true;
        public static final double kUpMotorManualVoltage = 10.0;
        public static final double kDownMotorManualVoltage = 10.0;
        public static final double[] kSpeakerShootRate = { 60.0, 60.0 };
        public static final double[] kAmpShootRate = { 30.0, 45.0 };
        public static final double[] kCarryShooterRate = { 28.0, 32.0 };
        public static final double kP = 0.0;
        public static final double kI = 0.0;
        public static final double kD = 0.0;
        public static final double kUpMotorS = 1.65;
        public static final double kUpMotorV = 0.123;
        public static final double kUpMotorA = 0.0;
        public static final double kDownMotorS = 0.681;
        public static final double kDownMotorV = 0.165;
        public static final double kDownMotorA = 0.0;
    }

    public static class TransportConstants {
        public static final int kTransportChannel = 22;
        public static final boolean kTransportInverted = true;
        public static final double kTransVoltage = 3;
        public static final double kReTransVoltage = -3;
        public static final double kDistanceRange = 3.5;
    }

    public static class IntakeConstants {
        public static final int kIntakeChannel = 24;
        public static final Boolean kIntakeInverted = true;
        public static final double kIntakeVoltage = 6;
        public static final double kThrowPrecentage = -4.0;
    }

    public static class RotateShooterConstants {
        public static final int kRotateShooterChannel = 21;
        public static final Boolean kRotateShooterInverted = true;
        public static final Boolean kEncoderInverted = true;
        public static final int kEncoderChannel = 2;
        public static final double kInitDegree = 57.0;
        public static final double kRotateVoltLimit = 5.0;
        public static final double kRotateAngleMin = 5.0;
        public static final double kRotateAngleMax = 65.0;
        public static final double kRotateDegreeErrorPoint = 3;
        public static final double kRotateAngleOffset = 0.52;
        public static final double kRotateTriggerValue = 0.15;
        public static final double kSpeakerHeight = 2.0;
        public static final double kShooterHeight = 0.31;
        public static final double kP = 0.095;
        public static final double kI = 0.0004;
        public static final double kD = 0.0002;
    }

    public static class RotateIntakeConstants {
        public static final int kRotateIntakeChannel = 29;
        public static final boolean kRotateIntakeInverted = false;
        public static final boolean kEncoderInverted = false;
        public static final int kRotateEncoderChannel = 4;
        public static final double kRotateIntakeMaxDegree = 89.0;
        public static final double kRotateIntakeMinDegree = 0.01;
        public static final double kRotateDownVoltage = 5.0;
        public static final double kRotateUpVoltage = -5.0;
        public static final double kP = 0.05;
        public static final double kI = 0.0;
        public static final double kD = 0.0;
        public static final double kInitDegree = 90.0;
        public static final double kUpDegree = 30.0;
        public static final double kDownDegree = 0.0;
        public static final double kMaxDegree = 92.0;
        public static final double kMinDegree = 0.0;
        public static final double kRotateVoltLimit = 5.0;
    }

    public static class HookConstants {
        public static final int kHookLineChannel = 23;
        public static final int kHookLeftMotorChannel = 28;
        public static final int kHookRightMotorChannel = 27;
        public static final boolean kHookMotorLeftInverted = true;
        public static final boolean kHookMotorRightInverted = false;
        public static final boolean kLineMotorInverted = false;
        public static final int kHookLeftEncoderChannel = 0;
        public static final int kHookLeftEncoderChannelB = 1;
        public static final int kHookRightEncoderChannelA = 2;
        public static final int kHookRightEncoderChannelB = 3;
        public static final double kP = 0.0;
        public static final double kI = 0.0;
        public static final double kD = 0.0;
        public static final double kLinePositionMax = 60.0;
        public static final double kLinePositionMin = 0.0;
        public static final double kLeftPositionMax = 60.0;
        public static final double kLeftPositionMin = 0.0;
        public static final double kRightPositionMax = 60.0;
        public static final double kRightPositionMin = 0.0;
        public static final double kHookleftToTopPositionMax = 45.0;
        public static final double kHookRightToTopPositionMax = 45.0;
        public static final double kHooklineToTopPositionMax = 45.0;
        public static final double kHookPositionConversionfactor = 1.0;
        public static final double kLinePower = 0;
        public static final double kHookMotorLeftVoltage = 0.0;
        public static final double kHookMotorRightVoltage = 0.0;
        public static final double kManualControlLineMotorPower = 0.25;
        public static final double kManualControlLeftHookMotorPower = 0.25;
        public static final double kManualControlRightHookMotorPower = 0.25;
        public static final double kInitSetpoint = 0.0;
        public static final double kLeftMotorModify = 0.0; // TO DO
        public static final double kRightMotorModify = 0.0; // TO DO
        public static final double kLineMotorModify = 0.0; // TO DO
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
        public static final double kFrontLeftCanCoderMagOffset = -0.079590;
        public static final double kFrontRightCanCoderMagOffset = -0.458984;
        public static final double kBackLeftCanCoderMagOffset = 0.355225;
        public static final double kBackRightCanCoderMagOffset = -0.333984;

        public static final double kMaxSpeed = 5;
        public static final double kMinSpeed = 0.25;
        public static final double kMinJoyStickValue = 0.3;
        public static final double kMaxAngularSpeed = 2.5 * Math.PI; // 1/2 rotation per second

        public static final double kXLimiterRateLimit = 3.0;
        public static final double kYLimiterRateLimit = 3.0;
        public static final double kRotLimiterRateLimit = 3.0;

        public static final boolean kFrontLeftDriveMotorInverted = true;
        public static final boolean kFrontRightDriveMotorInverted = false;
        public static final boolean kBackLeftDriveMotorInverted = true;
        public static final boolean kBackRightDriveMotorInverted = false;

        public static final boolean kGyroInverted = false; // wheather gyro is under the robot
        public static final double kGyroOffSet = 0;

        public static final double kDefaultMagnification = 0.8;
        public static final double kHighMagnification = 1.0;
    }

    public static final class ModuleConstants {
        public static final double kWheelRadius = 0.046;

        public static final double kWheelDiameterMeters = 0.15;

        public static final double kMaxModuleDriveVoltage = 8.0;

        public static final double kDriveClosedLoopRampRate = 0.8;// 1 second 1 unit
        public static final double kTurningClosedLoopRampRate = 0.25;

        public static final double kDesireSpeedtoMotorVoltage = kMaxModuleDriveVoltage / DrivebaseConstants.kMaxSpeed;

        public static final double kMaxModuleTuringVoltage = 5.0;

        public static final double kMaxSpeedTurningDegree = 180.0;

        public static final double kPRotationController = kMaxModuleTuringVoltage / kMaxSpeedTurningDegree;
        public static final double kIRotationController = 0.0;
        public static final double kDRotController = 0.0004;

        public static final boolean kTurningMotorInverted = true;
    }

    public static final class PowerDistributionConstants {
        public static final int kIntakeMotorCurrrentchannel = 0;
        public static final int kShooterDownMotorCurrentchannel = 1;
        public static final int kShooterUpMotorCurrentchannel = 2;
        public static final int kLineCurrentchannel = 3;
        public static final int kHookLeftMotorCurrentchannel = 4;
        public static final int kHookRightMotorCurrentchannel = 5;
        public static final int kTransportCurrentchannel = 0;
        public static final int kRiseShooterCurrentchannel = 1;
        public static final int kRotateIntakeCurrentchannel = 7;

        public static final double kIntakeMotorMaxCurrent = 40.0;
        public static final double kShooterDownMotorMaxCuurent = 40.0;
        public static final double kShooterUpMotorMaxCurrent = 40.0;
        public static final double kLineMotorMaxCurrent = 40.0;
        public static final double kHookMotor1MaxCurrent = 40.0;
        public static final double kHookMotor2MaxCurrent = 40.0;
        public static final double kTransportMaxCurrent = 40.0;
        public static final double kRotateShooterMaxCurrent = 40.0;
        public static final double kRotateIntakeMaxCurrent = 40.0;
    }

    public static final class AutoConstants {

        public static final String LBSToNote1 = "LBSToNote1";
        public static final String LBSToNote2 = "LBSToNote2";
        public static final String LBSToNote3 = "LBSToNote3";
        public static final String LBSToNote4 = "LBSToNote4";
        public static final String LBSToNote5 = "LBSToNote5";
        public static final String LBSToNote6 = "LBSToNote6";
        public static final String LBSToNote7 = "LBSToNote7";
        public static final String LBSToNote8 = "LBSToNote8";

        public static final String LTSToNote1 = "LTSToNote1";
        public static final String LTSToNote2 = "LTSToNote2";
        public static final String LTSToNote3 = "LTSToNote3";
        public static final String LTSToNote4 = "LTSToNote4";
        public static final String LTSToNote5 = "LTSToNote5";
        public static final String LTSToNote6 = "LTSToNote6";
        public static final String LTSToNote7 = "LTSToNote7";
        public static final String LTSToNote8 = "LTSToNote8";

        public static final String RBSToNote1 = "RBSToNote1";
        public static final String RBSToNote2 = "RBSToNote2";
        public static final String RBSToNote3 = "RBSToNote3";
        public static final String RBSToNote4 = "RBSToNote4";
        public static final String RBSToNote5 = "RBSToNote5";
        public static final String RBSToNote6 = "RBSToNote6";
        public static final String RBSToNote7 = "RBSToNote7";
        public static final String RBSToNote8 = "RBSToNote8";

        public static final String RTSToNote1 = "RTSToNote1";
        public static final String RTSToNote2 = "RTSToNote2";
        public static final String RTSToNote3 = "RTSToNote3";
        public static final String RTSToNote4 = "RTSToNote4";
        public static final String RTSToNote5 = "RTSToNote5";
        public static final String RTSToNote6 = "RTSToNote6";
        public static final String RTSToNote7 = "RTSToNote7";
        public static final String RTSToNote8 = "RTSToNote8";

        public static final String bottomRelayToLBS = "bottomRelayToLBS";
        public static final String bottomRelayToRBS = "bottomRelayToRBS";
        public static final String topRelayToLTS = "topRelayToLTS";
        public static final String topRelayToRTS = "topRelayToRTS";

        public static final double kPTranslation = 2.0;
        public static final double kITranslation = 0.001;
        public static final double kDTranslation = 0.01;
        public static final double kPRotation = 2.0;
        public static final double kIRotation = 0.001;
        public static final double kDRotation = 0.01;
        public static final double kMaxModuleSpeed = 3.36;
        public static final double kDrivebaseRadius = 2.17;

        public static final double kMaxVelocity = 3.36;
        public static final double kMaxAcceleration = 3.36;
        public static final double kMaxAngularVelocity = 453.38;
        public static final double kMaxAngularAcceleration = 453.38;
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
        public static final double pitchDegree = -20.0;
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
