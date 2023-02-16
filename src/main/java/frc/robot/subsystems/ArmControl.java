// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.InvertType;
import com.ctre.phoenix.motorcontrol.TalonFXInvertType;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.fasterxml.jackson.annotation.JacksonInject.Value;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class ArmControl extends SubsystemBase {
  
  /** CAN IDs  */
  public static WPI_TalonFX shoulderMotorRight = new WPI_TalonFX (Constants.kArm.kShoulderMotorIDRight); 
  public static WPI_TalonFX shoulderMotorLeft = new WPI_TalonFX (Constants.kArm.kShoulderMotorIDLeft);
  public static WPI_TalonSRX elbowMotorRight = new WPI_TalonSRX (Constants.kArm.kElbowMotorIDRight);
  public static WPI_TalonSRX elbowMotorLeft = new WPI_TalonSRX (Constants.kArm.kElbowMotorIDLeft);
  
  
  public ArmControl() {
    //Setting Shoulder inverted
    shoulderMotorLeft.setInverted(Constants.kArm.shoulderMotorLeftInvert);
    shoulderMotorRight.setInverted(Constants.kArm.shoulderMotorRightInvert);

    //Setting Sensor phase for shoulder
    shoulderMotorLeft.setSensorPhase(Constants.kArm.shoulderLeftPhase);
    shoulderMotorRight.setSensorPhase(Constants.kArm.shoulderRightPhase);

    //Shoulder PIDG
    shoulderMotorLeft.config_kP(0, Constants.kArm.kShoulderP);
    shoulderMotorLeft.config_kI(0, Constants.kArm.kShoulderI);
    shoulderMotorLeft.config_kD(0, Constants.kArm.kShoulderD);

    shoulderMotorRight.config_kP(0, Constants.kArm.kShoulderP);
    shoulderMotorRight.config_kI(0, Constants.kArm.kShoulderI);
    shoulderMotorRight.config_kD(0, Constants.kArm.kShoulderD);


    //Setting Elbow inverted
    elbowMotorLeft.setInverted(Constants.kArm.elbowMotorLeftInvert);
    elbowMotorRight.setInverted(Constants.kArm.elbowMotorRightInvert);
    
    //Setting Sensor phase for elbow
    elbowMotorLeft.setSensorPhase(Constants.kArm.elbowLeftPhase);
    elbowMotorRight.setSensorPhase(Constants.kArm.elbowRightPhase);
    
    //Elbow PIDG
    elbowMotorLeft.config_kP(0, Constants.kArm.kElbowP);
    elbowMotorLeft.config_kI(0, Constants.kArm.kElbowI);
    elbowMotorLeft.config_kD(0, Constants.kArm.kElbowD);
    
    elbowMotorRight.config_kP(0, Constants.kArm.kElbowP);
    elbowMotorRight.config_kI(0, Constants.kArm.kElbowI);
    elbowMotorRight.config_kD(0, Constants.kArm.kElbowD);
    
    
    elbowMotorRight.follow(elbowMotorLeft);
    shoulderMotorRight.follow(shoulderMotorLeft);

  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
