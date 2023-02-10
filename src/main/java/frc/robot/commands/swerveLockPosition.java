// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import com.ctre.phoenix.motorcontrol.NeutralMode;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.Swerve;
import frc.robot.subsystems.SwerveModule;

public class swerveLockPosition extends CommandBase {

  Swerve s_Swerve;
  double angle;

  /** Creates a new swerveLockPosition. */

  // this command locks out telop and locks the robot in its position by making
  // the wheels into an X. useful when you don't want to move.

  // Consider adding this to the telopDisable function.

  // @param angle: angle relative to the Y axis / axis parrallel to the 
  // ROBOT FRONT you want to lock the wheels to. in Degrees.
  public swerveLockPosition(Swerve s_Swerve, double angle) {
    // Use addRequirements() here to declare subsystem dependencies.

    this.s_Swerve = s_Swerve;
    this.angle = angle;

    addRequirements(s_Swerve);




  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {

    SwerveModuleState[] states = new SwerveModuleState[4];

    // remember that positive rotation goes counterclockwise while looking down at the robot.

    // front left
    states[0] = new SwerveModuleState(0, new Rotation2d(Units.degreesToRadians(angle + 45.0)));
    
    // front right
    states[1] = new SwerveModuleState(0, new Rotation2d(Units.degreesToRadians(angle - 45.0)));

    // rear left
    states[2] = new SwerveModuleState(0, new Rotation2d(Units.degreesToRadians(angle - 45.0)));
    
    // rear right
    states[3] =  new SwerveModuleState(0, new Rotation2d(Units.degreesToRadians(angle + 45.0)));

    s_Swerve.driveExtraManual(states);

    // Brake all the wheels
    for(SwerveModule mod:s_Swerve.mSwerveMods) {

      mod.mAngleMotor.setNeutralMode(NeutralMode.Brake);
      mod.mDriveMotor.setNeutralMode(NeutralMode.Brake);

    }


  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {

    for(SwerveModule mod:s_Swerve.mSwerveMods) {

      mod.mAngleMotor.setNeutralMode(Constants.Swerve.angleNeutralMode);
      mod.mDriveMotor.setNeutralMode(Constants.Swerve.driveNeutralMode);

    }


    // This can probably be gotten rid of.
    s_Swerve.resetModulesToAbsolute();

  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
