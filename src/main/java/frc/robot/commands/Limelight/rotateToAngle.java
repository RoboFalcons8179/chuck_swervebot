// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Limelight;

import java.util.List;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;
import frc.robot.Constants;

import frc.robot.subsystems.Swerve;

public class rotateToAngle extends SequentialCommandGroup {
  /** Creates a new doTrajectory. */

  public final static TrajectoryConfig config =
  new TrajectoryConfig(
          Constants.AutoConstants.kMaxSpeedMetersPerSecond,
          Constants.AutoConstants.kMaxAccelerationMetersPerSecondSquared)
      .setKinematics(Constants.Swerve.swerveKinematics);

  public rotateToAngle(Swerve s_Swerve, double target) {
    // Use addRequirements() here to declare subsystem dependencies.

    addRequirements(s_Swerve);

    var thetaController =
    new ProfiledPIDController(
        Constants.AutoConstants.kPThetaController, 0, 0, Constants.AutoConstants.kThetaControllerConstraints);
    thetaController.enableContinuousInput(-Math.PI, Math.PI);

    Trajectory rotateDegree =
    TrajectoryGenerator.generateTrajectory(
        // Start at the origin facing the +X direction
        new Pose2d(0, 0, s_Swerve.getPose().getRotation()),
        // Pass through these two interior waypoints, making an 's' curve path
        List.of(new Translation2d(0.1, 0), new Translation2d(-0.1, 0)),
        // End 3 meters straight ahead of where we started, facing forward
        new Pose2d(0, 0.0, new Rotation2d(Math.toRadians(target))),
        config);

    SwerveControllerCommand swerveControllerCommand =
        new SwerveControllerCommand(
            rotateDegree,
            s_Swerve::getPose,
            Constants.Swerve.swerveKinematics,
            new PIDController(Constants.AutoConstants.kPXController, 0, 0),
            new PIDController(Constants.AutoConstants.kPYController, 0, 0),
            thetaController,
            s_Swerve::setModuleStates,
            s_Swerve);



    addCommands(
    new InstantCommand(() -> s_Swerve.resetOdometry(rotateDegree.getInitialPose())),
    swerveControllerCommand);
    
  }

  

}
