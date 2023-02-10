package frc.robot.commands;

import static frc.robot.Constants.CamConstants;

import java.util.function.Supplier;

import org.photonvision.PhotonCamera;
import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.Swerve;


public class chaseTagV2 extends CommandBase {
  
      private static final TrapezoidProfile.Constraints X_CONSTRAINTS = new TrapezoidProfile.Constraints(Constants.CamConstants.kMaxSpeedMetersPerSecond, Constants.CamConstants.kMaxAccelerationMetersPerSecondSquared);
      private static final TrapezoidProfile.Constraints Y_CONSTRAINTS = new TrapezoidProfile.Constraints(Constants.CamConstants.kMaxSpeedMetersPerSecond, Constants.CamConstants.kMaxAccelerationMetersPerSecondSquared);
      private static final TrapezoidProfile.Constraints OMEGA_CONSTRAINTS =   new TrapezoidProfile.Constraints(Constants.CamConstants.kMaxAngularSpeedRadiansPerSecond, Constants.CamConstants.kMaxAngularSpeedRadiansPerSecondSquared);
      
      private final ProfiledPIDController xController = new ProfiledPIDController(Constants.CamConstants.LINEAR_P, Constants.CamConstants.LINEAR_I, Constants.CamConstants.LINEAR_D, X_CONSTRAINTS);
      private final ProfiledPIDController yController = new ProfiledPIDController(Constants.CamConstants.LINEAR_P, Constants.CamConstants.LINEAR_I, Constants.CamConstants.LINEAR_D, Y_CONSTRAINTS);
      private final ProfiledPIDController omegaController = new ProfiledPIDController(Constants.CamConstants.ANGULAR_P, Constants.CamConstants.ANGULAR_I, Constants.CamConstants.ANGULAR_D, OMEGA_CONSTRAINTS);

  private static int TAG_TO_CHASE;
  private static final Transform3d TAG_TO_GOAL = 
      new Transform3d(
          new Translation3d(Constants.CamConstants.GOAL_RANGE_METERS, 0.0, 0.0),
          new Rotation3d(0.0, 0.0, Math.PI));

  private final PhotonCamera photonCamera;
  private final Swerve drivetrainSubsystem;

  
  private PhotonTrackedTarget lastTarget;

  public boolean xAtGoal;
  public boolean yAtGoal;
  public boolean rAtGoal;
  public boolean lostTarget;


  public chaseTagV2(
        PhotonCamera photonCamera, 
        Swerve drivetrainSubsystem) {
    this.photonCamera = photonCamera;
    this.drivetrainSubsystem = drivetrainSubsystem;


    xController.setTolerance(0.2);
    yController.setTolerance(0.2);
    omegaController.setTolerance(Units.degreesToRadians(3));
    omegaController.enableContinuousInput(-Math.PI, Math.PI);

    var photonRes = photonCamera.getLatestResult();
    if (photonRes.hasTargets()) {

      TAG_TO_CHASE = photonRes.getBestTarget().getFiducialId();

    } else{}


  }

  @Override
  public void initialize() {
    lastTarget = null;
    var robotPose = drivetrainSubsystem.getPose();
    omegaController.reset(robotPose.getRotation().getRadians());
    xController.reset(robotPose.getX());
    yController.reset(robotPose.getY());



  }

  @Override
  public void execute() {
    var robotPose2d = new Pose2d();
    var robotPose = 
        new Pose3d(
            robotPose2d.getX(),
            robotPose2d.getY(),
            0.0, 
            new Rotation3d(0.0, 0.0, robotPose2d.getRotation().getRadians()));
    
    var photonRes = photonCamera.getLatestResult();

    
    if (photonRes.hasTargets()) {
      // Find the tag we want to chase
      var targetOpt = photonRes.getTargets().stream()
          .filter(t -> t.getFiducialId() == TAG_TO_CHASE)
          .filter(t -> !t.equals(lastTarget) && t.getPoseAmbiguity() <= .2 && t.getPoseAmbiguity() != -1)
          .findFirst();
      if (targetOpt.isPresent()) {
        var target = targetOpt.get();
        // This is new target data, so recalculate the goal
        lastTarget = target;
        
        // Transform the robot's pose to find the camera's pose
        var cameraPose = robotPose.transformBy(Constants.CamConstants.ROBOT_TO_CAMERA);

        // Trasnform the camera's pose to the target's pose
        var camToTarget = target.getBestCameraToTarget();
        var targetPose = cameraPose.transformBy(camToTarget);
        
        // Transform the tag's pose to set our goal
        var goalPose = targetPose.transformBy(TAG_TO_GOAL).toPose2d();

        // Drive
        xController.setGoal(goalPose.getX());
        yController.setGoal(goalPose.getY());
        omegaController.setGoal(goalPose.getRotation().getRadians());
      }
    }
    
    if (lastTarget == null) {
      // No target has been visible
      drivetrainSubsystem.stop();
      lostTarget = true;
    } else {
      // calculate
      var xSpeed = xController.calculate(robotPose.getX());
      var ySpeed = yController.calculate(robotPose.getY());
      var omegaSpeed = omegaController.calculate(robotPose2d.getRotation().getRadians());


      if(xAtGoal) {
        xSpeed = 0;
      }
      if(yAtGoal) {
        ySpeed = 0;
      }
      if(rAtGoal) {
        omegaSpeed = 0;
      }

      drivetrainSubsystem.drive(
         new Translation2d(xSpeed, ySpeed), omegaSpeed, false,true);



      // Troubleshoot

      
      SmartDashboard.putNumber("Robot range", robotPose.getX() );
      SmartDashboard.putNumber("ROBOT strafe",robotPose.getY());
      SmartDashboard.putNumber("ROBOT ANGLE", robotPose2d.getRotation().getRadians());

      SmartDashboard.putNumber("XSPEEDCMD", xSpeed);
      SmartDashboard.putNumber("YSPEEDCMD", ySpeed);
      SmartDashboard.putNumber("OMEGASPEEDCMD", omegaSpeed );

      SmartDashboard.putBoolean("X_CONTROLLER_AT_GOAL", xAtGoal);
      SmartDashboard.putBoolean("Y_CONTROLLER_AT_GOAL", yAtGoal);
      SmartDashboard.putBoolean("R_CONTROLLER_AT_GOAL", rAtGoal);

      SmartDashboard.putData(xController);
      SmartDashboard.putData(yController);
      SmartDashboard.putData(omegaController);


      
    }
  }

  @Override
  public void end(boolean interrupted) {
    drivetrainSubsystem.stop();
  }

  @Override
  public boolean isFinished() {

    return (xAtGoal && yAtGoal && rAtGoal) || lostTarget;

  }

}