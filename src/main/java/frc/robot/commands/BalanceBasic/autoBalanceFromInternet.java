// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

// Thanks to team 3683 !!!!

package frc.robot.commands.BalanceBasic;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Swerve;



public class autoBalanceFromInternet extends CommandBase {
  /** Creates a new autoBalanceFromInternet. */

  Swerve s_Swerve;
  private autoBalance autobalnce;
  double approachAngle;


  public autoBalanceFromInternet(Swerve s_Swerve, Double approachAngle) {

    this.s_Swerve     = s_Swerve;
    this.approachAngle = approachAngle;


    addRequirements(s_Swerve);


    
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    autobalnce = new autoBalance(s_Swerve);

  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    double speed = autobalnce.autoBalanceRoutine();
    // double angle = autobalnce.getDriveUpAngle();

    double angle = Math.toRadians(0);

    boolean correct = autobalnce.goToZeroAngle;

    if (correct) {
      s_Swerve.drive_Manually(speed, new Rotation2d(0));

    }
    else {
      s_Swerve.drive_Manually(speed, new Rotation2d(approachAngle));
    }


  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return autobalnce.getDone();
  }
}
