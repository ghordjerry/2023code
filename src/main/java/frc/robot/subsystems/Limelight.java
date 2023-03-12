// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.networktables.DoubleArraySubscriber;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.subsystems.Util.BotPoseFilter;
import edu.wpi.first.math.filter.MedianFilter;
import edu.wpi.first.networktables.DoubleArrayTopic;


public class Limelight extends SubsystemBase {
  
  NetworkTable table = NetworkTableInstance.getDefault().getTable("limelight");
  NetworkTableEntry tx = table.getEntry("tx");
  NetworkTableEntry ty = table.getEntry("ty");
  NetworkTableEntry ta = table.getEntry("ta");
  DoubleArraySubscriber posesub = table.getDoubleArrayTopic("botpose").subscribe(new double[] {});
  
    
  
  //read values periodically


  /** Creates a new ExampleSubsystem. */



  public Limelight() {

//post to smart dashboard periodically

  }

  /**
   * Example command factory method.
   *
   * @return a command
   */
  public CommandBase exampleMethodCommand() {
    // Inline construction of command goes here.
    // Subsystem::RunOnce implicitly requires `this` subsystem.
    return runOnce(
        () -> {
          /* one-time action goes here */
        });
  }

 
  public Pose3d getRobotPose() {

    double[] result = posesub.get();
    Translation3d tran3d = new Translation3d(result[0], result[1], result[2]);
  Rotation3d r3d = new Rotation3d(result[3], result[4], result[5]);
  Pose3d p3d = new Pose3d(tran3d, r3d);

    return p3d;
}


  /**
   * An example method querying a boolean state of the subsystem (for example, a digital sensor).
   *
   * @return value of some boolean subsystem state, such as a digital sensor.
   */
  public boolean exampleCondition() {
    // Query some boolean state, such as a digital sensor.
    return false;
  }

  @Override
  public void periodic() {
//read values periodically
double x = tx.getDouble(0.0);
double y = ty.getDouble(0.0);
double area = ta.getDouble(0.0);
double[] result = posesub.get();


//post to smart dashboard periodically
SmartDashboard.putNumber("LimelightX", x);
SmartDashboard.putNumber("LimelightY", y);
SmartDashboard.putNumber("LimelightArea", area);
SmartDashboard.putNumber("xTranslation", result[0]);
SmartDashboard.putNumber("yTranslation", result[1]);
SmartDashboard.putNumber("zTranslation", result[2]);
SmartDashboard.putNumber("xRotation", result[3]);
SmartDashboard.putNumber("yRotation", result[4]);
SmartDashboard.putNumber("zRotation", result[5]);


  }  

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }
}