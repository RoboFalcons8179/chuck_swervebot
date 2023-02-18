// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.DemandType;
import com.ctre.phoenix.motorcontrol.InvertType;
import com.ctre.phoenix.motorcontrol.NeutralMode;
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
  public static WPI_TalonSRX elbowMotorLeft = new WPI_TalonSRX (Constants.kArm.kElbowMotorIDLeft);
  public static WPI_TalonFX grabberMotor = new WPI_TalonFX (Constants.kArm.kGrabberMotorID);
  
  
  public ArmControl() {
    //Setting Shoulder inverted//
    shoulderMotorLeft.setInverted(Constants.kArm.shoulderMotorLeftInvert);
    shoulderMotorRight.setInverted(Constants.kArm.shoulderMotorRightInvert);

    // Shoulder Motor Break Mode//
    shoulderMotorLeft.setNeutralMode(NeutralMode.Brake);
    shoulderMotorRight.setNeutralMode(NeutralMode.Brake);

    //Setting Sensor phase for shoulder//
    shoulderMotorLeft.setSensorPhase(Constants.kArm.shoulderLeftPhase);
    shoulderMotorRight.setSensorPhase(Constants.kArm.shoulderRightPhase);

    //Shoulder PIDG//
    shoulderMotorLeft.config_kP(0, Constants.kArm.kShoulderP);
    shoulderMotorLeft.config_kI(0, Constants.kArm.kShoulderI);
    shoulderMotorLeft.config_kD(0, Constants.kArm.kShoulderD);

    shoulderMotorRight.config_kP(0, Constants.kArm.kShoulderP);
    shoulderMotorRight.config_kI(0, Constants.kArm.kShoulderI);
    shoulderMotorRight.config_kD(0, Constants.kArm.kShoulderD);


    //Setting Elbow inverted//
    elbowMotorLeft.setInverted(Constants.kArm.elbowMotorLeftInvert);
    
    //Setting Sensor phase for elbow//
    elbowMotorLeft.setSensorPhase(Constants.kArm.elbowLeftPhase);

    // Elbow Motor Break mode
    elbowMotorLeft.setNeutralMode(NeutralMode.Brake);
    
    //Elbow PIDG//
    elbowMotorLeft.config_kP(0, Constants.kArm.kElbowP);
    elbowMotorLeft.config_kI(0, Constants.kArm.kElbowI);
    elbowMotorLeft.config_kD(0, Constants.kArm.kElbowD);

    //Setting Grabber Inverted//
    grabberMotor.setInverted(Constants.kArm.grabberMotorInvert);

    // Setting Grabber Sensor Phase//
    grabberMotor.setSensorPhase(Constants.kArm.grabberMotorPhase);

    // Grabber PID//
    grabberMotor.config_kP(0, Constants.kArm.kGrabberP);
    grabberMotor.config_kP(0, Constants.kArm.kGrabberI);
    grabberMotor.config_kP(0, Constants.kArm.kGrabberD);
    
    shoulderMotorRight.follow(shoulderMotorLeft);

  }


  public void goToShoulderSetpoint(double point) {

    shoulderMotorLeft.set(ControlMode.Position, point, DemandType.ArbitraryFeedForward , shoulderAuxInputGrav);

  }

  public double shoulderAuxInputGrav;
  public double elbowAuxInputGrav;

  public double shoulderEncodertoDegrees(double reading) {
    /* Turns shoulder encoder reading to a degree output */

    double out = (90.0 - 0.0) * (reading / (Constants.kArm.kShoulderEncoderAt90Degrees - 0.0));
    return out;

  }

  public double elbowEncodertoDegrees(double reading) {
    /* Turns elbow encoder reading to a degree output */

    double out = (90.0 - 0.0) * (reading / (Constants.kArm.kElbowEncoderAt90Degrees - 0.0));
    return out;

  }


  @Override
  public void periodic() {

    // accound for gravity
    double shoulderAngle = shoulderMotorLeft.getSelectedSensorPosition();
    double elbowAngle = elbowMotorLeft.getSelectedSensorPosition();

    double shoulderAngleDegree = shoulderEncodertoDegrees(shoulderAngle);
    double elbowAngleDegree = elbowEncodertoDegrees(elbowAngle);

    double elbowRelitiveToGroundAngle = 360.0 - 90.0 - shoulderAngleDegree - elbowAngleDegree;    

    double elbowAuxInputGrav = Constants.kArm.kElbowG * Math.cos(Math.toRadians(elbowRelitiveToGroundAngle));

    double shoulderAuxInputGrav = Constants.kArm.kShoulderG * Math.sin(Math.toRadians(shoulderAngle));

    // still need to account for second link on 


    // This method will be called once per scheduler run
  }
}
