// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

public class Elevator extends SubsystemBase {
  
private WPI_TalonFX mElevator = new WPI_TalonFX(25);
  /** Creates a new elevator. */

  public Elevator() {

    mElevator.configFactoryDefault();
    
  // m_slaveTalon.configFactoryDefault();
    mElevator.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor, 0, 1);
    mElevator.setInverted(true);
    mElevator.setSensorPhase(false);
    mElevator.setNeutralMode(NeutralMode.Brake);
    mElevator.config_kF(0, 0.04, 10);
    mElevator.config_kP(0, 0.01, 10);

    
    mElevator.enableVoltageCompensation(false);
  //mElevator.configVoltageCompSaturation(9, 1);
    mElevator.configMotionCruiseVelocity(9192*5);   // start testing with half the value of maximum sensor velocity
    mElevator.configMotionAcceleration(4096*5);
    mElevator.configMotionSCurveStrength(0);  
    mElevator.setSelectedSensorPosition(-800);
  }


 
  public void stop(){
    mElevator.set(ControlMode.PercentOutput,0);
  }
  public void hmlevel(){
    mElevator.set(ControlMode.MotionMagic,170500);
  }

  public void level3(){

    mElevator.set(ControlMode.MotionMagic, 138000); 
  }
  public void level4(){
    mElevator.set(ControlMode.MotionMagic, 141000); 
  }

  public void level2(){
    mElevator.set(ControlMode.MotionMagic,72000);
  }

  // public void level1(){

  //   mElevator.set(ControlMode.MotionMagic,8000);

  // }

  public void elevatorstop(){
    mElevator.set(ControlMode.PercentOutput,0);

  }

  public void back(){

    mElevator.set(ControlMode.MotionMagic,0);
  }
  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
