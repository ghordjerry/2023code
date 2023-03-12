// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.sensors.Pigeon2Configuration;
import com.ctre.phoenix.sensors.WPI_Pigeon2;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.commands.PPSwerveControllerCommand;

import edu.wpi.first.math.controller.HolonomicDriveController;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.ModuleConstants;
import frc.robot.subsystems.Util.AimUtil;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

public class DriveSubsystem extends SubsystemBase {
  
  // Robot swerve modules
  private final SwerveModule m_frontLeft =
      new SwerveModule(
          DriveConstants.kFrontLeftDriveMotorPort,
          DriveConstants.kFrontLeftTurningMotorPort,
          DriveConstants.kFrontLeftTurningMotorEncoderChannel,
          DriveConstants.kFrontLeftDriveEncoderReversed,
          DriveConstants.kFrontLeftTurningEncoderReversed,
          DriveConstants.kfrontleftturn,
          DriveConstants.kfrontleftdrive,
          ModuleConstants.kfrontleftcancoderOffset,
          0.12,
          -0.12);

  private final SwerveModule m_frontRight =
      new SwerveModule(
          DriveConstants.kFrontRightDriveMotorPort,
          DriveConstants.kFrontRightTurningMotorPort,
          DriveConstants.kFrontRightTurningMotorEncoderChannel,
          DriveConstants.kFrontRightDriveEncoderReversed,
          DriveConstants.kFrontRightTurningEncoderReversed,
          DriveConstants.kfontrightturn,
          DriveConstants.kfrontrightdrive,
          ModuleConstants.kfrontrightcancoderOffset,
          0.12,
          -0.12);        

   private final SwerveModule m_rearRight =
      new SwerveModule(
          DriveConstants.kRearRightDriveMotorPort,
          DriveConstants.kRearRightTurningMotorPort,
          DriveConstants.kRearRightTurningMotorEncoderChannel,
          DriveConstants.kRearRightDriveEncoderReversed,
          DriveConstants.kRearRightTurningEncoderReversed,
          DriveConstants.kOutputRever4,
          DriveConstants.kDriveReverse4,
          ModuleConstants.krearrightcancoderOffset,
          0.12,
          -0.12);
                  
  private final SwerveModule m_rearLeft =
      new SwerveModule(
          DriveConstants.kRearLeftDriveMotorPort,
          DriveConstants.kRearLeftTurningMotorPort,
          DriveConstants.kRearLeftTurningMotorEncoderChannel,
          DriveConstants.kRearLeftDriveEncoderReversed,
          DriveConstants.kRearLeftTurningEncoderReversed,
          DriveConstants.kOutputRever3,
          DriveConstants.kDriveReverse3,
          ModuleConstants.krearleftcancoderoffset,
          0.12,
          -0.12);


  // The gyro sensor
  private final WPI_Pigeon2 m_gyro = new WPI_Pigeon2(DriveConstants.kPigeon2Port,"CANivore");
  // Odometry class for tracking robot pose
  SwerveDriveOdometry m_odometry =
      new SwerveDriveOdometry(
          DriveConstants.kDriveKinematics,
          m_gyro.getRotation2d(),
          new SwerveModulePosition[] {
            m_frontLeft.getPosition(),
            m_frontRight.getPosition(),
            m_rearLeft.getPosition(),
            m_rearRight.getPosition()
          });

  /** Creates a new DriveSubsystem. */
  public DriveSubsystem() {
    m_gyro.configFactoryDefault();
    Pigeon2Configuration config = new Pigeon2Configuration();
    config.MountPosePitch = 0;
    config.MountPoseRoll = 0;
    config.MountPoseYaw = 0;
    config.EnableCompass = false;
    config.DisableNoMotionCalibration=false;
    config.DisableTemperatureCompensation=false;
    config.enableOptimizations= false;

    m_gyro.configAllSettings(config);
    m_gyro.setYaw(0);
  }

  @Override
  public void periodic() {
    // Update the odometry in the periodic block
    m_odometry.update(
        m_gyro.getRotation2d(),
        new SwerveModulePosition[] {
          m_frontLeft.getPosition(),
          m_frontRight.getPosition(),
          m_rearLeft.getPosition(),
          m_rearRight.getPosition()
        });

    // SmartDashboard.putNumber("X", m_odometry.getPoseMeters().getX());
    // SmartDashboard.putNumber("Y", m_odometry.getPoseMeters().getY());
    // SmartDashboard.putNumber("Yaw", m_gyro.getYaw());
    // SmartDashboard.putNumber("error", m_frontLeft.getError());
    SmartDashboard.putNumber("frontleft", m_frontLeft.getTurningEncoderAngle());
    SmartDashboard.putNumber("frontright", m_frontRight.getTurningEncoderAngle());
    SmartDashboard.putNumber("rearright", m_rearRight.getTurningEncoderAngle());
    SmartDashboard.putNumber("rearleft", m_rearLeft.getTurningEncoderAngle());

  }
  public boolean isLevel() {
    return Math.abs(m_gyro.getPitch()) < 2 && Math.abs(m_gyro.getRoll()) < 2;
  }

  /**
   * Returns the currently-estimated pose of the robot.
   *
   * @return The pose.
   */
  public Pose2d getPose() {
    //return null;
    return m_odometry.getPoseMeters();
  }  
  
  public void zeroyaw() {
    m_gyro.setYaw(0);
  }




  /**
   * Resets the odometry to the specified pose.
   *
   * @param pose The pose to which to set the odometry.
   */
  public void resetOdometry(Pose2d pose) {
    m_odometry.resetPosition(
        m_gyro.getRotation2d(),
        new SwerveModulePosition[] {
          m_frontLeft.getPosition(),
          m_frontRight.getPosition(),
          m_rearLeft.getPosition(),
          m_rearRight.getPosition()
        },
        pose);
  }
  // SlewRateLimiter limiter1 = new SlewRateLimiter(1.5, -1.5, 0);
  // SlewRateLimiter limiter2 = new SlewRateLimiter(1.5, -1.5, 0);

  public Runnable drive;

  /**
   * Method to drive the robot using joystick info.
   *
   * @param xSpeed Speed of the robot in the x direction (forward).
   * @param ySpeed Speed of the robot in the y direction (sideways).
   * @param rot Angular rate of the robot.
   * @param fieldRelative Whether the provided x and y speeds are relative to the field.
   */
  public void drive(double xSpeed, double ySpeed, double rot, boolean fieldRelative) {
    // deadzone
    if(Math.abs(xSpeed) < 0.1 && Math.abs(ySpeed) < 0.1 && Math.abs(rot) < 0.25) {
      xSpeed = 0;
      ySpeed = 0;
      rot = 0;
    }
    // xSpeed = limiter1.calculate(xSpeed);
    // ySpeed = limiter2.calculate(ySpeed);
    

    var swerveModuleStates =
        DriveConstants.kDriveKinematics.toSwerveModuleStates(
            fieldRelative
                ? ChassisSpeeds.fromFieldRelativeSpeeds(xSpeed, ySpeed, rot, m_gyro.getRotation2d())
                : new ChassisSpeeds(xSpeed, ySpeed, rot));
  
    setModuleStates(swerveModuleStates);
    // SmartDashboard.putNumber("error", xSpeed - m_frontLeft.getDriveEncoderVelocity());
  }

  /**
   * Sets the swerve ModuleStates.
   *
   * @param desiredStates The desired SwerveModule states.
   */
  public void setModuleStates(SwerveModuleState[] desiredStates) {
    SwerveDriveKinematics.desaturateWheelSpeeds(
        desiredStates, DriveConstants.kMaxSpeedMetersPerSecond);
    m_frontLeft.setDesiredState(desiredStates[0]);
    m_frontRight.setDesiredState(desiredStates[1]);
    m_rearLeft.setDesiredState(desiredStates[2]);
    m_rearRight.setDesiredState(desiredStates[3]);
  }

  /** Resets the drive encoders to currently read a position of 0. */
  public void resetEncoders() {
    m_frontLeft.resetEncoders();
    m_rearLeft.resetEncoders();
    m_frontRight.resetEncoders();
    m_rearRight.resetEncoders();
  }

// Assuming this method is part of a drivetrain subsystem that provides the necessary methods
public Command followTrajectoryCommand(PathPlannerTrajectory traj, boolean isFirstPath) {
  return new SequentialCommandGroup(
       new InstantCommand(() -> {
         // Reset odometry for the first path you run during auto
         if(isFirstPath){
             this.resetOdometry(traj.getInitialHolonomicPose());
         }
       }),
       new PPSwerveControllerCommand(
           traj, 
           this::getPose, // Pose supplier
           DriveConstants.kDriveKinematics, // SwerveDriveKinematics
           new PIDController(0, 0, 0), // X controller. Tune these values for your robot. Leaving them 0 will only use feedforwards.
           new PIDController(0, 0, 0), // Y controller (usually the same values as X controller)
           new PIDController(0, 0, 0), // Rotation controller. Tune these values for your robot. Leaving them 0 will only use feedforwards.
           this::setModuleStates, // Module states consumer
           false, // Should the path be automatically mirrored depending on alliance color. Optional, defaults to true
           this // Requires this drive subsystem
       )
   );
}



double x, y;
  public void level(){

    if (m_gyro.getRoll() > 2){
      y=-1;
    } 
    else if (m_gyro.getRoll() <-2){
      y=1;
    }
    else y=0;
  
    if (m_gyro.getPitch() > 2){
      x=-1;
    }
    else if (m_gyro.getPitch() <-2){
      x=1;
    }
    else x=0;

      drive(y, x, 0, true);
    }

  public boolean isleveled(){
    return Math.abs(m_gyro.getRoll())<2 && Math.abs(m_gyro.getPitch())<2;
  }

  /** Zeroes the heading of the robot. */
  public void zeroHeading() {
    m_gyro.reset();
  }

  /**
   * Returns the heading of the robot.
   *
   * @return the robot's heading in degrees, from -180 to 180
   */
  public double getHeading() {
    return m_gyro.getRotation2d().getDegrees();
  }

  /**
   * Returns the turn rate of the robot.
   *
   * @return The turn rate of the robot, in degrees per second
   */
  
   public HolonomicDriveController controller = new HolonomicDriveController(
    new PIDController(0.14, 0, 0.24), //x
    new PIDController(0.8, 0, 0.187), //y
    new ProfiledPIDController(0.13, 0, 0, AutoConstants.kThetaControllerConstraints));
    
  public double getTurnRate() {
    return m_gyro.getRate() * (DriveConstants.kGyroReversed ? -1.0 : 1.0);
  } public void leftTarget() {
    Pose2d curr = getPose();
    double y = AimUtil.getLeftTargetY(curr);
    Pose2d target = new Pose2d(curr.getX(), y, Rotation2d.fromDegrees(0));
    var chassisSpeed = controller.calculate(curr, target, 0.4, curr.getRotation());

    drive(chassisSpeed.vxMetersPerSecond, 
          chassisSpeed.vyMetersPerSecond, 
          chassisSpeed.omegaRadiansPerSecond, 
          false);
  }

  public void rightTarget() {
    Pose2d curr = getPose();
    double y = AimUtil.getRightTargetY(curr);
    Pose2d target = new Pose2d(curr.getX(), y, Rotation2d.fromDegrees(0));
    var chassisSpeed = controller.calculate(curr, target, 0.4, curr.getRotation());

    drive(chassisSpeed.vxMetersPerSecond, 
          chassisSpeed.vyMetersPerSecond, 
          chassisSpeed.omegaRadiansPerSecond, 
          false);
  }

  public void midTarget(){
    Pose2d curr = getPose();
    double y =AimUtil.getMidTargetY(curr);
    Pose2d target =new Pose2d(curr.getX(), y, Rotation2d.fromDegrees(0));
    var chassisSpeed = controller.calculate(curr, target, 0.4, curr.getRotation());

    drive(chassisSpeed.vxMetersPerSecond,
          chassisSpeed.vyMetersPerSecond,
          chassisSpeed.omegaRadiansPerSecond,
          false);
  }
}
