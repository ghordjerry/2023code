// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.trajectory.TrapezoidProfile;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {

  public static final class OIConstants {    
    public static final int kJoystickPort = 0;
    public static final int kDriverstationPort = 1;
  }

  public static final class IntakeConstants{
    public static final int cubeintakeport = 9;
    public static final int coneintakeport = 10;
    public static final int turnport = 11;
    public static final double turnkP = 0;
    public static final double turnkI = 0;
    public static final double turnkD = 0;
  }

  public static final class DriveConstants {
    
    public static final int kPigeon2Port = 13;

    public static final int kFrontLeftDriveMotorPort = 1;
    public static final int kRearLeftDriveMotorPort = 7;
    public static final int kFrontRightDriveMotorPort = 3;
    public static final int kRearRightDriveMotorPort = 5;

    public static final int kFrontLeftTurningMotorPort = 2;
    public static final int kRearLeftTurningMotorPort = 8;
    public static final int kFrontRightTurningMotorPort = 4;
    public static final int kRearRightTurningMotorPort = 6;

    public static final int kFrontLeftTurningMotorEncoderChannel = 9;
    public static final int kRearLeftTurningMotorEncoderChannel = 12;
    public static final int kFrontRightTurningMotorEncoderChannel = 10;
    public static final int kRearRightTurningMotorEncoderChannel = 11;

    // turning encoder
    public static final boolean kFrontLeftTurningEncoderReversed = true;
    public static final boolean kRearLeftTurningEncoderReversed = true;
    public static final boolean kFrontRightTurningEncoderReversed = true;
    public static final boolean kRearRightTurningEncoderReversed = true;

    // turning motor
    public static final boolean kfrontleftturn = true;
    public static final boolean kfontrightturn = true;
    public static final boolean kOutputRever3 = true;
    public static final boolean kOutputRever4 = true;

    // drive motor
    public static final boolean kfrontleftdrive = true;
    public static final boolean kfrontrightdrive = false;
    public static final boolean kDriveReverse3 = false;
    public static final boolean kDriveReverse4 = true;
    
    // drive encoder
    public static final boolean kFrontLeftDriveEncoderReversed = false;
    public static final boolean kRearLeftDriveEncoderReversed = false;
    public static final boolean kFrontRightDriveEncoderReversed = false;
    public static final boolean kRearRightDriveEncoderReversed = false;

    public static final double kTrackWidth = 0.6;
    // Distance between centers of right and left wheels on robot
    public static final double kWheelBase = 0.6;
    // Distance between front and back wheels on robot
    public static final SwerveDriveKinematics kDriveKinematics =
        new SwerveDriveKinematics(
            new Translation2d(kWheelBase / 2, kTrackWidth / 2),
            new Translation2d(kWheelBase / 2, -kTrackWidth / 2),
            new Translation2d(-kWheelBase / 2, kTrackWidth / 2),
            new Translation2d(-kWheelBase / 2, -kTrackWidth / 2));

    public static final boolean kGyroReversed = false;

    // These are example values only - DO NOT USE THESE FOR YOUR OWN ROBOT!
    // These characterization values MUST be determined either experimentally or theoretically
    // for your robot's drive.
    // The SysId tool provides a convenient method for obtaining these values for your robot.
    public static final double ksVolts = 1;
    public static final double kvVoltSecondsPerMeter = 0.8;
    public static final double kaVoltSecondsSquaredPerMeter = 0.15;

    public static final double kMaxSpeedMetersPerSecond = 3;
  }

  public static final class ModuleConstants {
    public static final double kMaxModuleAngularSpeedRadiansPerSecond = 2 * Math.PI;
    public static final double kMaxModuleAngularAccelerationRadiansPerSecondSquared = 2 * Math.PI;

    public static final int kEncoderCPR = 2048;
    // public static final double mk4igearrate = 21.428;
    public static final double kDriveCoefficient =
        0.1 * Math.PI / kEncoderCPR / 8.14;

    public static final double kPModuleTurningController = 1;

    public static final double kPModuleDriveController = 1;

    public static final double kfrontleftcancoderOffset = 40.868; 
    public static final double kfrontrightcancoderOffset = 134.825; 
    public static final double krearrightcancoderOffset = 65.039;
    public static final double krearleftcancoderoffset =-50.536;
  }

  public static final class AutoConstants {
    public static final double kMaxSpeedMetersPerSecond = 3;
    public static final double kMaxAccelerationMetersPerSecondSquared = 3;
    public static final double kMaxAngularSpeedRadiansPerSecond = Math.PI;
    public static final double kMaxAngularSpeedRadiansPerSecondSquared = Math.PI;

    public static final double kPXController = 0.1;
    public static final double kPYController = 0.1;
    public static final double kPThetaController = 1;

    // Constraint for the motion profiled robot angle controller
    public static final TrapezoidProfile.Constraints kThetaControllerConstraints =
        new TrapezoidProfile.Constraints(
            kMaxAngularSpeedRadiansPerSecond, kMaxAngularSpeedRadiansPerSecondSquared);
  }
}