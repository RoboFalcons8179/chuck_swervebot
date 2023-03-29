// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.*;


public class zerolime extends CommandBase {
  /** Creates a new zero. */

  Swerve swerve;
  Limelight lime;
  boolean isDone;

  public zerolime(Swerve swerve, Limelight lime) {

    this.swerve = swerve;
    this.lime = lime;

    addRequirements(swerve, lime);
    
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {

    // lime.reflect_init();

    isDone = false;

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


      double target = 0;

      double distanceFromLimeLightTarget = lime.get_reflectTX();


      
      double speed = -0.5;
      double slowspeedscale = 0.5;

      if (distanceFromLimeLightTarget > target) {
        speed = speed * -1;
      }

      // System.out.println("-------------------------");

      // System.out.println(distanceFromLimeLightTarget);
      // System.out.println(speed);

      // System.out.println(Math.abs(distanceFromLimeLightTarget - target));




      // check our gyro

      if ((Math.abs(distanceFromLimeLightTarget - target) > 5)) {

        swerve.drive(new Translation2d(0, speed), 0 , true, false);
      }
      else if ((Math.abs(distanceFromLimeLightTarget - target) > 1.0)) {

        swerve.drive(new Translation2d(0, slowspeedscale*speed), 0, true, false);
      
      }

      else{
        // debounce, then end command.
        swerve.stop();
        isDone = true;
        this.cancel();
      }

    //  System.out.println("-------------------------");



  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {

    // stop your drivetrain
    swerve.stop();

    lime.goToDriverCam();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {

    // if we are close enough to our target

    return isDone;
  }
}
