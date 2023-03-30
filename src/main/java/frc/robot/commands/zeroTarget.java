// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import com.kauailabs.navx.IMUProtocol.GyroUpdate;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.interfaces.Gyro;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.commands.*;
import frc.robot.subsystems.*;


public class zeroTarget extends CommandBase {
  /** Creates a new zero. */

  Swerve swerve;
  boolean isDone;

  Rotation2d targetR;
  PIDController tcalc;

  boolean special;


  public zeroTarget(Swerve swerve, Rotation2d targetR) {

    this.swerve = swerve;
    this.targetR = targetR;

    addRequirements(swerve);
    
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {

    isDone = false;


    tcalc = new PIDController(1, 0, 0);
    tcalc.enableContinuousInput(-180,180);

    tcalc.setSetpoint(targetR.getDegrees());

    special = false;

    if (targetR.getDegrees() < -175 || targetR.getDegrees() > 175) {
      special = true;
    }


  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    /* tell the swerve to drive in your:

        // figure out if you need to turn right or left. multiply your nominal Rotation speed by -1 or 1
    
        public void drive(Translation2d translation, double rotation, boolean fieldRelative, boolean isOpenLoop) {

            Translation 2d: new Translation2d(0,0)
            roatition: rotation speed, + or minus depending on the direction
            fieldRelitive: True - we want to rotate to a specific field orientation
            isOpnLoop: false (maybe true)
        
        Decided if we are close enough to the gyro point, then stop.
     */

    //  System.out.println("-------------------------");

      double target = this.targetR.getDegrees();

      double currentAngle = swerve.gyro.getYaw();


      // if (target < 181 && target > 179) {
        
      //   if (currentAngle < 0) {

      //     currentAngle = currentAngle + 180;
      //     target = 0;


      //   }
      // }


      
      double speed = 1.5;
      double slowspeedscale = 0.33;

      double direction = tcalc.calculate(currentAngle);

      tcalc.reset();


      if (!special && currentAngle > target) {
        speed = speed * -1;
      }


      else if (special && direction < 0) {
        speed = speed * -1;
        System.out.println(direction);

      }


      // System.out.println(currentAngle);
      // System.out.println(speed);

      // System.out.println(Math.abs(currentAngle - target));




      // check our gyro

      if ((Math.abs(currentAngle - target) > 30)) {

        swerve.drive(new Translation2d(0,0), speed , true, false);
      }
      else if ((Math.abs(currentAngle - target) > 2)) {

        swerve.drive(new Translation2d(0,0), slowspeedscale * speed, true, false);
      
      }

      else{
        // debounce, then end command.
        swerve.stop();
        isDone = true;
        this.cancel();

      }




  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {

    // stop your drivetrain

    System.out.println("Rotation Done!!");
    swerve.stop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {

    // if we are close enough to our target

    return isDone;
  }
}
