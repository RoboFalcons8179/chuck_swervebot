// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.Swerve;

public class balanceWrapper extends CommandBase {
  /** Creates a new balanceWrapper. */

  private double direction;
  private Swerve s_Swerve;

  double magnitude;
  double gyroAccel;
  double power = Constants.kBalance.power;

  double speedScale;

  boolean quickOut = false;


  public balanceWrapper(Swerve s_Swerve, double speedScale) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.s_Swerve = s_Swerve;
    this.speedScale = speedScale;
    addRequirements(s_Swerve);


  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    direction = s_Swerve.pointingUpAngle();
    
    if (s_Swerve.isRobotLevel()) {
      quickOut = true;
    }

  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    s_Swerve.drive_Manually(power*speedScale, new Rotation2d(direction));

    // power = power * 0.997;


    double pitch = Math.abs(s_Swerve.gyro.getPitch());
    double roll = Math.abs(s_Swerve.gyro.getRoll());

    magnitude = Math.sqrt(Math.pow(pitch, 2) + Math.pow(roll, 2));


    // double xGyro = s_Swerve.gyro.getRawGyroX();
    // double yGyro = s_Swerve.gyro.getRawGyroY();

    // gyroAccel = Math.sqrt(Math.pow(xGyro, 2) + Math.pow(yGyro, 2));

    System.out.println(power);

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {

    s_Swerve.stop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    
    if (magnitude < Constants.kBalance.MagnitudeAngle) {
      return true;
    }

    if (Math.abs(power) < 0.002) {
      return true;
    }
    // if (gyroAccel > 5) {
    //   return true;
    // }


    return quickOut;
  }
}
