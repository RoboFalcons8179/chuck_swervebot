// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Limelight;
import frc.robot.subsystems.Swerve;

public class zeroTag extends CommandBase {
  /** Creates a new zeroTag. */

  Swerve swerve;
  Limelight lime;

  
  public zeroTag(Swerve swerve, Limelight lime) {

    this.swerve = swerve;
    this.lime = lime;

    addRequirements(swerve, lime);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {

    lime.april_init();

  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    double target = 0;

    double distanceFromLimeLightTarget = lime.get_reflectTX();


    
    double speed = -0.5;
    double slowspeedscale = 0.15;

    if (distanceFromLimeLightTarget > target) {
      speed = speed * -1;
    }

    System.out.println("==============");

    System.out.println(distanceFromLimeLightTarget);
    System.out.println(speed);

    System.out.println(Math.abs(distanceFromLimeLightTarget - target));


    if ((Math.abs(distanceFromLimeLightTarget - target) > 6)) {

      swerve.drive(new Translation2d(0, speed), 0 , true, false);
    }
    else if ((Math.abs(distanceFromLimeLightTarget - target) > 2)) {

      swerve.drive(new Translation2d(0, slowspeedscale*speed), 0, true, false);
    
    }

    else{
      // debounce, then end command.
      swerve.stop();
      this.cancel();
    }

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
    return false;
  }
}
