// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.revrobotics.CANSparkBase.IdleMode;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.util.Units;



/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
  public static final class ControllerConstants {
    public static final int kNavigatorPort = 0;
    public static final int kOperatorPort = 1;
    public static final double kDriveDeadband = 0.1;
  }

  public static final class ClimberConstants {
    public static final int kRightClimberCanID = 11;
    public static final int kLeftClimberCanID = 12;
    
    public static double kClimberPower = 0.75;
  }

  public static final class IntakeConstants {
    public static final int kUpperIntakeCanID = 15;
    public static final int kLowerIntakeCanID = 16;

    public static double kIntakePower = 0.85;
  }

  public static final class ShooterConstants {
    public static final int kRightShooterCanID = 20;
    public static final int kLeftShooterCanID = 21;
    
    public static double kShooterPower = 0.85;
    public static double kShooterAmpPower = 0.175;
    public static double kShooterIntakePower = -0.175;

    // public static double kShooterRPM = 4500;
    // public static double kShooterAmpRPM = 800;
    // public static double kShooterIntakeRPM = 1000;

    // public static double kShooterP = 0.025;
    // public static double kShooterI = 0.0;
    // public static double kShooterD = 0.005;
    // public static double kShooterFF = 0;
    // public static double kShooterIz = 0;
    // public static final double kShooterMaxOutput = 1;
    // public static final double kShooterMinOutput = -1;

  }

  public static final class VisionConstans {
    public static final String kCameraName = "Arducam_OV9281_USB_Camera";
        // Cam mounted facing forward, half a meter forward of center, half a meter up from center.
        public static final Transform3d kRobotToCam =
                new Transform3d(new Translation3d(0.5, 0.0, 0), new Rotation3d(0, 0, 0));

        // The layout of the AprilTags on the field
        public static final AprilTagFieldLayout kTagLayout =
                AprilTagFields.kDefaultField.loadAprilTagLayoutField();

        // The standard deviations of our vision estimated poses, which affect correction rate
        // (Fake values. Experiment and determine estimation noise on an actual robot.)
        public static final Matrix<N3, N1> kSingleTagStdDevs = VecBuilder.fill(4, 4, 8);
        public static final Matrix<N3, N1> kMultiTagStdDevs = VecBuilder.fill(0.5, 0.5, 1);

        public static final double kVisionP = 0.28;
        public static final double kVisionD = 0.0;
  }

  public static final class DriveConstants {
    // Driving Parameters - Note that these are not the maximum capable speeds of
    // the robot, rather the allowed maximum speeds
    public static final double kMaxSpeedMetersPerSecond = 4.8;
    public static final double kMaxAngularSpeed = 2 * Math.PI; // radians per second

    public static final double kDirectionSlewRate = 1.2; // radians per second
    public static final double kMagnitudeSlewRate = 1.8; // percent per second (1 = 100%)
    public static final double kRotationalSlewRate = 2.0; // percent per second (1 = 100%)

    // Chassis configuration
    public static final double kTrackWidth = Units.inchesToMeters(25);
    // Distance between centers of right and left wheels on robot
    public static final double kWheelBase = Units.inchesToMeters(25);
    // Distance between front and back wheels on robot
    public static final double kDriveBaseRadius = Math.sqrt(kWheelBase * kWheelBase + kTrackWidth * kTrackWidth) / 2;
    // Distance between center of robot to center of wheelbase
    public static final SwerveDriveKinematics kDriveKinematics = new SwerveDriveKinematics(
        new Translation2d(kWheelBase / 2, kTrackWidth / 2),
        new Translation2d(kWheelBase / 2, -kTrackWidth / 2),
        new Translation2d(-kWheelBase / 2, kTrackWidth / 2),
        new Translation2d(-kWheelBase / 2, -kTrackWidth / 2));

    // Angular offsets of the modules relative to the chassis in radians
    public static final double kFrontLeftChassisAngularOffset = -Math.PI / 2;
    public static final double kFrontRightChassisAngularOffset = 0;
    public static final double kBackLeftChassisAngularOffset = Math.PI;
    public static final double kBackRightChassisAngularOffset = Math.PI / 2;

    // SPARK MAX CAN IDs
    public static final int kFrontLeftDrivingCanId = 1;
    public static final int kFrontLeftTurningCanId = 2;
    public static final int kFrontRightDrivingCanId = 3;
    public static final int kFrontRightTurningCanId = 4;
    public static final int kRearLeftDrivingCanId = 5;
    public static final int kRearLeftTurningCanId = 6;
    public static final int kRearRightDrivingCanId = 7;
    public static final int kRearRightTurningCanId = 8;

    // Pigeon Gyro
    public static final int kPigeonIMUId = 20;
    public static final boolean kGyroReversed = true;
  }

  public static final class ModuleConstants {
    // The MAXSwerve module can be configured with one of three pinion gears: 12T, 13T, or 14T.
    // This changes the drive speed of the module (a pinion gear with more teeth will result in a
    // robot that drives faster).
    public static final int kDrivingMotorPinionTeeth = 14;

    // Invert the turning encoder, since the output shaft rotates in the opposite direction of
    // the steering motor in the MAXSwerve Module.
    public static final boolean kTurningEncoderInverted = true;

    // Calculations required for driving motor conversion factors and feed forward
    public static final double kDrivingMotorFreeSpeedRps = NeoMotorConstants.kFreeSpeedRpm / 60;
    public static final double kWheelDiameterMeters = 0.0762;
    public static final double kWheelCircumferenceMeters = kWheelDiameterMeters * Math.PI;
    // 45 teeth on the wheel's bevel gear, 22 teeth on the first-stage spur gear, 15 teeth on the bevel pinion
    public static final double kDrivingMotorReduction = (45.0 * 22) / (kDrivingMotorPinionTeeth * 15);
    public static final double kDriveWheelFreeSpeedRps = (kDrivingMotorFreeSpeedRps * kWheelCircumferenceMeters)
        / kDrivingMotorReduction;

    public static final double kDrivingEncoderPositionFactor = (kWheelDiameterMeters * Math.PI)
        / kDrivingMotorReduction; // meters
    public static final double kDrivingEncoderVelocityFactor = ((kWheelDiameterMeters * Math.PI)
        / kDrivingMotorReduction) / 60.0; // meters per second

    public static final double kTurningEncoderPositionFactor = (2 * Math.PI); // radians
    public static final double kTurningEncoderVelocityFactor = (2 * Math.PI) / 60.0; // radians per second

    public static final double kTurningEncoderPositionPIDMinInput = 0; // radians
    public static final double kTurningEncoderPositionPIDMaxInput = kTurningEncoderPositionFactor; // radians

    public static final double kDrivingP = 0.04;
    public static final double kDrivingI = 0;
    public static final double kDrivingD = 0;
    public static final double kDrivingFF = 1 / kDriveWheelFreeSpeedRps;
    public static final double kDrivingMinOutput = -1;
    public static final double kDrivingMaxOutput = 1;

    public static final double kTurningP = 1;
    public static final double kTurningI = 0;
    public static final double kTurningD = 0;
    public static final double kTurningFF = 0;
    public static final double kTurningMinOutput = -1;
    public static final double kTurningMaxOutput = 1;

    public static final IdleMode kDrivingMotorIdleMode = IdleMode.kBrake;
    public static final IdleMode kTurningMotorIdleMode = IdleMode.kBrake;

    public static final int kDrivingMotorCurrentLimit = 50; // amps
    public static final int kTurningMotorCurrentLimit = 20; // amps
  }

  public static final class DrivetrainConstants {
    public static final int kLeftSRXDrivePort = 51;
    public static final int kLeftSPXDrivePort = 02;
    public static final int kRightSRXDrivePort = 20;
    public static final int kRightSPXDrivePort = 21;

    public static final double kDriveForwardMultiplier = 1;
    public static final double kDriveTurnMultiplier = 1;

    public static final double kLinearKS = 1.5819;
    public static final double kLinearKV = 2.9238;
    public static final double kLinearKA = 1.6274;
    public static final double kAngularKS = 1.9351;
    public static final double kAngularKV = 3.4356;
    public static final double kAngularKA = 1.2996;

    public static final double kP = 1.5551;
    public static final double kI = 0.0;
    public static final double kD = 0.0;
    public static final double kWheelDiameterInches = 6;
    public static final double kWheelCircumferenceMeters = Units.inchesToMeters(6 * Math.PI);
    public static final double kTrackwidth = 0.57591;
    public static final DifferentialDriveKinematics kDriveKinematics =
        new DifferentialDriveKinematics(kTrackwidth);
    public static final double kCimToWheelGearing = 10.71;

    public static final double kAngularKP = .1;
    public static final double kAngularKD = .0;

    public static final double kMaxTurnRate = 100;
    public static final double kMaxTurnAccel = 10;
    public static final double kDegreeTolerance = 0.75;
    public static final double kTurnRateTolerance = 1.5;

    public static final double kAutoMaxSpeed = 1;
    public static final double kAutoMaxAccel = 1;

    public static final double kAutoTime = 2.5;
  }

  public static final class NeoMotorConstants {
    public static final double kFreeSpeedRpm = 5676;
  }
}
