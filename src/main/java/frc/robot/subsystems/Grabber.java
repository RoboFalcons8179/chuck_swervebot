// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.LimitSwitchNormal;
import com.ctre.phoenix.motorcontrol.LimitSwitchSource;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Grabber extends SubsystemBase {
  /** Creates a new Grabber. */
  public static WPI_TalonSRX grabberMotor = new WPI_TalonSRX (Constants.kGrabber.kGrabberMotorID);
  
  public Grabber() {

    //Setting Grabber Inverted//
    grabberMotor.setInverted(Constants.kGrabber.grabberMotorInvert);

    // Setting Grabber Sensor Phase//
    grabberMotor.setSensorPhase(Constants.kGrabber.grabberMotorPhase);

    // Grabber PID//
    grabberMotor.config_kP(0, Constants.kGrabber.kGrabberP);
    grabberMotor.config_kI(0, Constants.kGrabber.kGrabberI);
    grabberMotor.config_kD(0, Constants.kGrabber.kGrabberD); 
    grabberMotor.config_kF(0, Constants.kGrabber.kGrabberF);

    //Starting Position//
    grabberMotor.setNeutralMode(NeutralMode.Brake);

    //Limiting the Current//
    grabberMotor.enableCurrentLimit(true);
    grabberMotor.configContinuousCurrentLimit(Constants.kGrabber.ampLimit, Constants.kGrabber.timeout);

    // Limit switch
    grabberMotor.configForwardLimitSwitchSource(LimitSwitchSource.FeedbackConnector, LimitSwitchNormal.NormallyOpen);

  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public void setSpeed(double amps) {
    grabberMotor.set(ControlMode.Current, amps);
  }

  public void openClawSubsystem() {

    // Open until we hit the back limit switch
    grabberMotor.set(ControlMode.PercentOutput, Constants.kGrabber.backPower);
    

  }

  public void closeClawSubsystem() {

    // close onto our current
    grabberMotor.set(ControlMode.Current, Constants.kGrabber.squeezeCurrent);


  }

  public void stop() {
    grabberMotor.set(0);
  }
}
