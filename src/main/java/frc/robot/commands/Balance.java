// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.Swerve;


import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.math.geometry.Rotation2d;


public class Balance extends CommandBase {

  private Swerve s_Swerve;
  private AHRS gyro;

  /** Creates a new Balance. */
  public Balance(Swerve s_Swerve) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.s_Swerve = s_Swerve;
    gyro = s_Swerve.gyro;
    
    // addRequirements(s_Swerve);


    
  }

  private double angleTarget;
  private double backwards;
  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    
    angleTarget = s_Swerve.pointingUpAngle();
    // if (angleTarget > 90 && angleTarget < 270){
    //   backwards = 1; 
    // }
    // else{
    //   backwards = -1;
    // }
    if(gyro.getRawAccelY() < 0){
      backwards=-1;
    }
    else{
      backwards=1;
    }
    if (angleTarget < 0){
      angleTarget=angleTarget+Math.toRadians(180);
    }

  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    
    s_Swerve.drive_Manually(Constants.kBalance.power*backwards, new Rotation2d(angleTarget));
    

    

    // s_Swerve.drive( 
    //   new Translation2d(Constants.kBalance.power, new Rotation2d(s_Swerve.pointingUpAngle())),
    //   0,
    //   false,
    //   false);


  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {

    if (interrupted == false) { // This means that it thinks it is level

    }


  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    // boolean isDone = s_Swerve.isRobotLevel(); // ALL ACCELERATION IN G's
    return false;
  }
}
