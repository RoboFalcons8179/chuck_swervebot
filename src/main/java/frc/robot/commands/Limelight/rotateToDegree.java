// // Copyright (c) FIRST and other WPILib contributors.
// // Open Source Software; you can modify and/or share it under the terms of
// // the WPILib BSD license file in the root directory of this project.

// package frc.robot.commands.Limelight;

// import java.util.List;

// import edu.wpi.first.math.controller.PIDController;
// import edu.wpi.first.math.controller.ProfiledPIDController;
// import edu.wpi.first.math.geometry.Pose2d;
// import edu.wpi.first.math.geometry.Rotation2d;
// import edu.wpi.first.math.geometry.Translation2d;
// import edu.wpi.first.math.trajectory.Trajectory;
// import edu.wpi.first.math.trajectory.TrajectoryConfig;
// import edu.wpi.first.math.trajectory.TrajectoryGenerator;
// import edu.wpi.first.wpilibj2.command.CommandBase;
// import edu.wpi.first.wpilibj2.command.InstantCommand;
// import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;
// import frc.robot.Constants;
// import frc.robot.subsystems.Swerve;

// public class rotateToDegree extends CommandBase {
//   /** Creates a new rotateToZero. */

//   private ProfiledPIDController thetaController;
//   private double difference;

//   private Swerve s_Swerve;
//   private double target;
//   private double deadband = 2;
//   boolean isDone = false;

//   public final static TrajectoryConfig config =
//   new TrajectoryConfig(
//           Constants.AutoConstants.kMaxSpeedMetersPerSecond,
//           Constants.AutoConstants.kMaxAccelerationMetersPerSecondSquared)
//       .setKinematics(Constants.Swerve.swerveKinematics);


// public final static TrajectoryConfig slowAndSteady = 
//   new TrajectoryConfig(1, 2)
//       .setKinematics(Constants.Swerve.swerveKinematics);


//   public rotateToDegree(Swerve s_Swerve, double target) {

//     addRequirements(s_Swerve);

//     Trajectory rotateDegree =
//     TrajectoryGenerator.generateTrajectory(
//         // Start at the origin facing the +X direction
//         new Pose2d(0, 0, s_Swerve.getPose().getRotation()),
//         // Pass through these two interior waypoints, making an 's' curve path
//         List.of(new Translation2d(0.1, 0), new Translation2d(-0.1, 0)),
//         // End 3 meters straight ahead of where we started, facing forward
//         new Pose2d(0, 0.0, new Rotation2d(Math.toRadians(target))),
//         config);


//     var thetaController =
//     new ProfiledPIDController(
//         Constants.AutoConstants.kPThetaController, 0, 0, Constants.AutoConstants.kThetaControllerConstraints);
    
//     thetaController.enableContinuousInput(-Math.PI, Math.PI);

//     SwerveControllerCommand swerveControllerCommand =
//         new SwerveControllerCommand(
//           rotateDegree,
//             s_Swerve::getPose,
//             Constants.Swerve.swerveKinematics,
//             new PIDController(Constants.AutoConstants.kPXController, 0, 0),
//             new PIDController(Constants.AutoConstants.kPYController, 0, 0),
//             thetaController,
//             s_Swerve::setModuleStates,
//             s_Swerve);



//     addCommands(
//     new InstantCommand(() -> s_Swerve.resetOdometry(traj.getInitialPose())),
//     swerveControllerCommand);


//   }

//   // Called when the command is initially scheduled.
//   @Override
//   public void initialize() {

//   }

//   // Called every time the scheduler runs while the command is scheduled.
//   @Override
//   public void execute() {

//     // double power = 0.2;

//     // if (s_Swerve.getPose().getRotation().getDegrees() > (target + deadband)) {
//     //   power = power * -1;
//     // }
//     // else if (s_Swerve.getPose().getRotation().getDegrees() < (target - deadband)) {
//     //   power = power;
//     // } 
//     // else{
//     //   power = 0;
//     //   isDone = true;
//     // }

//     // s_Swerve.drive(new Translation2d(0,0),  power, false, true);


//   }

//   // Called once the command ends or is interrupted.
//   @Override
//   public void end(boolean interrupted) {}

//   // Returns true when the command should end.
//   @Override
//   public boolean isFinished() {
 
//     return isDone;
//   }
// }
