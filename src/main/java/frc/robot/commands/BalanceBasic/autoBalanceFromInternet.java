// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.BalanceBasic;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Swerve;



public class autoBalanceFromInternet extends CommandBase {
  /** Creates a new autoBalanceFromInternet. */

  Swerve s_Swerve;
  private autoBalance autobalnce;


  public autoBalanceFromInternet(Swerve s_Swerve) {

    this.s_Swerve = s_Swerve;
    addRequirements(s_Swerve);

    autobalnce = new autoBalance(s_Swerve.gyro);

    
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    double speed = autobalnce.autoBalanceRoutine();

    s_Swerve.drive_Manually(speed, new Rotation2d(Math.PI));

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
