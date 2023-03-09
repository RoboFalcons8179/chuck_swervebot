// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.ResourceBundle.Control;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.DemandType;
import com.ctre.phoenix.motorcontrol.InvertType;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.TalonFXInvertType;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class ArmControl extends SubsystemBase {
  
  /** CAN IDs  */
  public static WPI_TalonFX shoulderMotorRight = new WPI_TalonFX (Constants.kArm.kShoulderMotorIDRight); 
  public static WPI_TalonFX shoulderMotorLeft = new WPI_TalonFX (Constants.kArm.kShoulderMotorIDLeft);
  public static WPI_TalonSRX elbowMotorLeft = new WPI_TalonSRX (Constants.kArm.kElbowMotorID);

  // Starting position relitive to the straigt out form.
  public double startPositionto90Elbow = -2269 * 3; // Note that the *3 is from the gear ratio. It was not measured.
  public double startPositionto90Shoulder = 45803;
  
  
  public ArmControl() {

    shoulderMotorLeft.setSelectedSensorPosition(0);
    shoulderMotorRight.setSelectedSensorPosition(0);
    elbowMotorLeft.setSelectedSensorPosition(0);


    //Setting Shoulder inverted//
    shoulderMotorLeft.setInverted(Constants.kArm.shoulderMotorLeftInvert);
    shoulderMotorRight.setInverted(Constants.kArm.shoulderMotorRightInvert);

    // Shoulder Motor Break Mode//
    shoulderMotorLeft.setNeutralMode(NeutralMode.Brake);
    shoulderMotorRight.setNeutralMode(NeutralMode.Brake);

    //Setting Sensor phase for shoulder//
    shoulderMotorLeft.setSensorPhase(Constants.kArm.shoulderLeftPhase);
    shoulderMotorRight.setSensorPhase(Constants.kArm.shoulderRightPhase);


    //Shoulder PIDF//
    shoulderMotorLeft.config_kP(0, Constants.kArm.kShoulderP);
    shoulderMotorLeft.config_kI(0, Constants.kArm.kShoulderI);
    shoulderMotorLeft.config_kD(0, Constants.kArm.kShoulderD);
    shoulderMotorLeft.config_kF(0, Constants.kArm.kShoulderF);

    shoulderMotorRight.config_kP(0, Constants.kArm.kShoulderP);
    shoulderMotorRight.config_kI(0, Constants.kArm.kShoulderI);
    shoulderMotorRight.config_kD(0, Constants.kArm.kShoulderD);
    shoulderMotorRight.config_kF(0, Constants.kArm.kShoulderF);

    shoulderMotorLeft.configMaxIntegralAccumulator(0, 100000);




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
    elbowMotorLeft.config_kF(0, Constants.kArm.kElbowF);

    elbowMotorLeft.configMotionCruiseVelocity(Constants.kArm.elbowVel);
    elbowMotorLeft.configMotionAcceleration(Constants.kArm.elbowAccel);

    
    
    shoulderMotorRight.follow(shoulderMotorLeft);

  }


  //// MOVING COMMANDS

  // These are the last commanded positions. This is so when we don't tell the arm what to do,
  // it knows where it should be and can adjust.
  public double lastElbowPosition = 0;
  public double lastShoulderPosition = 0;

  public double shoulderAuxInputGrav = 0; // The arb inputs to account for gravity. they get updated every robot cycle.
  public double elbowAuxInputGrav = 0; 
  
  
  public void holdPosition() {
    // This is the default command to hold position. It WILL NOT be perfect, but it will be good enough to stay still enough.

    boolean activeHold = true;
    //TODO make boolean for seeing if arm is inside robot if this is true make no grav compensation//
    //TODO increase the value of the error zone of the shoulder control loop, increase the maximum intergal accumalation//

    // change this if you want to actively hold/float the values or
    // just have the arm sit there.

    if(activeHold){
      shoulderMotorLeft.set(ControlMode.PercentOutput, 0, DemandType.ArbitraryFeedForward, shoulderAuxInputGrav);//this is redundant since left is set to follow right?
      shoulderMotorRight.set(ControlMode.PercentOutput, 0);
      //elbowMotorLeft.set(ControlMode.PercentOutput, 0, DemandType.ArbitraryFeedForward, elbowAuxInputGrav);
      elbowMotorLeft.set(ControlMode.MotionMagic, elbowAngle2encoder(90+50), DemandType.ArbitraryFeedForward, elbowAuxInputGrav);

    }
    else {
      shoulderMotorLeft.set(0);
      shoulderMotorRight.set(0);

      elbowMotorLeft.set(0);
    }
  }

  public double shoulderCurrentAngle(){

    double shoulderAngle = shoulderMotorLeft.getSelectedSensorPosition();
  
      double shoulderAngleDegree = shoulderEncoder2Angle(shoulderAngle);

      return shoulderAngleDegree;
  }

  public double elbowCurrentAngle(){
      double elbowAngle = elbowMotorLeft.getSelectedSensorPosition();
  
      double elbowAngleDegree = elbowEncoder2Angle(elbowAngle);

      return elbowAngleDegree;
  }

  public boolean panelMoveDecision (double panelDegree) {
    if (panelDegree > -0.01 ){return true;}
    else  {return false;}
    
  }

  public boolean panelMove (double panelDegree) {
    if (panelDegree > -0.01 && panelDegree < -0.01 ){return true;}
    else {return false;}
    
  }

  public void goToShoulderSetpoint(double degree) {
    /* USE THIS FUNCTION TO COMMAND SETPOINTS
     * DO NOT - AND I MEAN DO NOT ACCESS THE MOTORS 
     * THEMSELVES
     * 
     * THAT MEANS YOU MATTHEW
     */
    shoulderMotorLeft.set(ControlMode.MotionMagic, 
      shoulderAngle2encoder(degree),
      DemandType.ArbitraryFeedForward,
      shoulderAuxInputGrav);
  }

  public void goToElbowSetpoint(double degree) {
    /* USE THIS FUNCTION TO COMMAND SETPOINTS
     * DO NOT - AND I MEAN DO NOT ACCESS THE MOTORS 
     * THEMSELVES
     * 
     * THAT MEANS YOU MATTHEW
     */
    elbowMotorLeft.set(ControlMode.MotionMagic, 
      elbowAngle2encoder(degree),
      DemandType.ArbitraryFeedForward,
      elbowAuxInputGrav);
  }

  
  public double elbowAngle2encoder(double degrees) {
    // mapping our desired anglular input to the right angular input.

    // Matching zeros for elbow: 0 degrees is equal to our experimental value. See the 
    // spreadsheet.


    /*
     *  y - y1 = m * (x-x1)
     * 
     * y  = desired encoder val
     * y1 = encoder val at a known point (constant)
     * m  =  slope, in encoder tics per degree (constant)
     * x  = our input
     * x1 = degree value at known point (constant)
     * 
     * See the spreadsheet.
     */

     final double y1 = startPositionto90Elbow;
     final double x1 = 0; // we are defining sticking straigt out as 0 degrees.
     final double m = 2006 * 3 / 90; // FROM TIM: The *3 is from the gear ratio

     return (m * (degrees - x1)) + y1;


  }

  public double shoulderAngle2encoder(double degrees) {
    // mapping our desired anglular input to the right angular input.
    // see function above

    final double y1 = startPositionto90Shoulder;
    final double x1 = 90; // we are defining sticking straigt out as +90 degrees.
    final double m = startPositionto90Shoulder / 90; // units encoder tics per degres.

    // Two points are (0,0) and (90,startPositionto90Shoulder)

    return (m * (degrees - x1)) + y1;

    
  }

  public double elbowEncoder2Angle(double tic) {

    // reverse of the functions above. 

    // x = (y-y1)/m + x1
    final double y1 = startPositionto90Elbow;
    final double x1 = 0; // we are defining sticking straigt out as 0 degrees.
    final double m = 2006 * 3 / 90; // FROM TIM: The *3 is from the gear ratio
    
    return (tic - y1)/m + x1;

  }
  
  public double shoulderEncoder2Angle(double tic) {

    final double y1 = startPositionto90Shoulder;
    final double x1 = 90; // we are defining sticking straigt out as +90 degrees.
    final double m = startPositionto90Shoulder / 90; // units encoder tics per degres.

    return (tic - y1)/m + x1;

  }





  @Override
  public void periodic() {

    calculateGravAuxInput();
    //setNoGoZones();


  }

  void calculateGravAuxInput() {

        // account for gravity
    // why are these not instance / static variables instead of just periodic ? -z
      // these are more for looking at the math. I don't care that they go away - I only care
      // about the output grav aux.
      double shoulderAngle = shoulderMotorLeft.getSelectedSensorPosition();
      double elbowAngle = elbowMotorLeft.getSelectedSensorPosition();
  
      double shoulderAngleDegree = shoulderEncoder2Angle(shoulderAngle);
      double elbowAngleDegree = elbowEncoder2Angle(elbowAngle);
  
      double elbowRelitiveToGroundAngle = 360.0 - 90.0 - shoulderAngleDegree - elbowAngleDegree;    
  
      this.elbowAuxInputGrav = Constants.kArm.kElbowG * Math.cos(Math.toRadians(elbowRelitiveToGroundAngle));
      this.shoulderAuxInputGrav = Constants.kArm.kShoulderG * Math.sin(Math.toRadians(shoulderAngle));

  }

  private void setNoGoZones() { // TODO SET UP ANGLE LIMITS AND CHECK POLARITIES
    /* sets up software limit switched to limit the elbow moving 
        while moving through the middle */

    double shoulderAngleInTheFrame = 10; 
    // degrees, TO DO angle that the shoulder is in the frame

    double minElbowAngleInFrame = 90.0 + 30.0; 
    // degrees from zero, TO DO min angle of the elbow required to enter the robot frame. TO DO

    double maxShoulderAngle = 100;
    // degrees,  TO DO +/- the max shoulder angle

    double maxElbowAngle = 165;
    // degrees,  TO DO +/- the max elbow angle


    double shoulderloation = shoulderEncoder2Angle(shoulderMotorLeft.getSelectedSensorPosition());
    double elbowlocation = elbowEncoder2Angle(elbowMotorLeft.getSelectedSensorPosition());

    boolean isFront = shoulderloation > 0; // is the shoulder in front
    boolean isTop = elbowlocation > 0; // is the elbow on the top or bottom

    boolean elbowAngleAllowedToEnter = Math.abs(elbowlocation) > minElbowAngleInFrame;


    if ( -shoulderAngleInTheFrame < shoulderloation && shoulderloation < shoulderAngleInTheFrame ) { 
        
        //aka inside the robot frame. we want to add software limits to make
        // sure that we cant drive the claw into the robot.

        if (isTop && !elbowAngleAllowedToEnter) {

          elbowMotorLeft.configReverseSoftLimitThreshold(elbowAngle2encoder(minElbowAngleInFrame));

        }
        else if(!isTop && !elbowAngleAllowedToEnter) {
          
          elbowMotorLeft.configForwardSoftLimitThreshold(elbowAngle2encoder(-minElbowAngleInFrame));

        } else {

          // aka we can move the elbow wherever we want
          elbowMotorLeft.configReverseSoftLimitThreshold(elbowAngle2encoder(-maxElbowAngle));
          elbowMotorLeft.configForwardSoftLimitThreshold(elbowAngle2encoder(+maxElbowAngle));

        }

        // if we are in the frame, no addl limits for the shoulder movement.
        shoulderMotorLeft.configReverseSoftLimitThreshold(-maxShoulderAngle);
        shoulderMotorLeft.configReverseSoftLimitThreshold(+maxShoulderAngle);

    }
    else { // aka shoulder outside the robot frame
      // here we want to make sure we do not enter the robot frame while
      // our elbow is not in a valid location.

      if (isFront && !elbowAngleAllowedToEnter) {
        shoulderMotorLeft.configReverseSoftLimitThreshold(shoulderAngle2encoder(shoulderAngleInTheFrame));
      } 
      else if (!isFront && !elbowAngleAllowedToEnter) {

        shoulderMotorLeft.configForwardSoftLimitThreshold(shoulderAngle2encoder(-shoulderAngleInTheFrame));
      } 
      else {

        // the elbow is at a good angle to move the shoulder into the bot.

        shoulderMotorLeft.configReverseSoftLimitThreshold(-maxShoulderAngle);
        shoulderMotorLeft.configReverseSoftLimitThreshold(+maxShoulderAngle);
        
      }

      elbowMotorLeft.configReverseSoftLimitThreshold(elbowAngle2encoder(-maxElbowAngle));
      elbowMotorLeft.configForwardSoftLimitThreshold(elbowAngle2encoder(+maxElbowAngle));

    }

  }
}
