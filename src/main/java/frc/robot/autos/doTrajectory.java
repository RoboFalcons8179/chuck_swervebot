// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.autos;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;
import frc.robot.Constants;

import frc.robot.subsystems.Swerve;

public class doTrajectory extends SequentialCommandGroup {
  /** Creates a new doTrajectory. */
  public doTrajectory(Swerve s_Swerve, Trajectory traj) {
    // Use addRequirements() here to declare subsystem dependencies.

    addRequirements(s_Swerve);

    var thetaController =
    new ProfiledPIDController(
        0.13, 0, 0, Constants.AutoConstants.kThetaControllerConstraints);
    thetaController.enableContinuousInput(-Math.PI, Math.PI);

    SwerveControllerCommand swerveControllerCommand =
        new SwerveControllerCommand(
            traj,
            s_Swerve::getPose,
            Constants.Swerve.swerveKinematics,
            new PIDController(3.5, 0, 0.005),
            new PIDController(3.5, 0, 0.005),
            thetaController,
            s_Swerve::setModuleStates,
            s_Swerve);



    addCommands(
    new InstantCommand(() -> s_Swerve.resetOdometry(traj.getInitialPose())),
    swerveControllerCommand);
    
  }

  

}
