// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import org.photonvision.PhotonUtils;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.Constants.Swerve.*;
import frc.robot.subsystems.Swerve;
import frc.robot.subsystems.pVision;

public class chaseTag extends CommandBase {

  private Swerve s_Swerve;
  private pVision vision;

  Translation2d setDriveTranslate;
  Double setDriveRotation;

  public int tgtNum;

  public final Pose2d targetPose2d = new Pose2d(new Translation2d(Constants.CamConstants.GOAL_RANGE_METERS, Constants.CamConstants.GOAL_OFSET_METERS), 
    new Rotation2d(Constants.CamConstants.GOAL_ANGLE_RAD));


  public boolean done;




      // controllers for auto movement
      private static final TrapezoidProfile.Constraints X_CONSTRAINTS = new TrapezoidProfile.Constraints(Constants.AutoConstants.kMaxSpeedMetersPerSecond, Constants.AutoConstants.kMaxAccelerationMetersPerSecondSquared);
      private static final TrapezoidProfile.Constraints Y_CONSTRAINTS = new TrapezoidProfile.Constraints(Constants.AutoConstants.kMaxSpeedMetersPerSecond, Constants.AutoConstants.kMaxAccelerationMetersPerSecondSquared);
      private static final TrapezoidProfile.Constraints OMEGA_CONSTRAINTS =   new TrapezoidProfile.Constraints(Constants.AutoConstants.kMaxAngularSpeedRadiansPerSecond, Constants.AutoConstants.kMaxAngularSpeedRadiansPerSecond);
      
      private final ProfiledPIDController xController = new ProfiledPIDController(Constants.CamConstants.LINEAR_P, Constants.CamConstants.LINEAR_I, Constants.CamConstants.LINEAR_D, X_CONSTRAINTS);
      private final ProfiledPIDController yController = new ProfiledPIDController(Constants.CamConstants.LINEAR_P, Constants.CamConstants.LINEAR_I, Constants.CamConstants.LINEAR_D, Y_CONSTRAINTS);
      private final ProfiledPIDController omegaController = new ProfiledPIDController(Constants.CamConstants.ANGULAR_P, Constants.CamConstants.ANGULAR_I, Constants.CamConstants.ANGULAR_D, OMEGA_CONSTRAINTS);

      public boolean xAtGoal;
      public boolean yAtGoal;
      public boolean rAtGoal;
      


  /** Creates a new chaseTag. */
  public chaseTag(Swerve ins_Swerve, pVision invision) {

    done = false;

    this.s_Swerve = ins_Swerve;
    this.vision = invision;

    addRequirements(s_Swerve, vision);

    






    // this will get enabled when we are close to a tag. therefore we 
    // are going to save the tag number.




    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {

    setDriveRotation = 0.0;
    setDriveTranslate = new Translation2d(0,0);

    xController.setTolerance(0.2);
    yController.setTolerance(0.2);
    omegaController.setTolerance(Units.degreesToRadians(3));
    omegaController.enableContinuousInput(-Math.PI, Math.PI);

    xAtGoal = false;
    yAtGoal = false;
    rAtGoal = false;

    
    var results = vision.camera.getLatestResult();


    if(!results.hasTargets()) {
      done = true;
    } 
    else { // there are no targets - end this command

      tgtNum = results.getBestTarget().getFiducialId();

    // get initial readings
    double rangeDif = PhotonUtils.calculateDistanceToTargetMeters(
      Constants.CamConstants.CAMERA_HEIGHT_METERS,
      Constants.CamConstants.TARGET_HEIGHT_METERS,
      Constants.CamConstants.CAMERA_PITCH_RADIANS,
      Units.degreesToRadians(results.getBestTarget().getPitch()));

    Translation2d robotPose = new Translation2d(rangeDif, 
      new Rotation2d(Units.degreesToRadians( results.getBestTarget().getYaw())));


      // reset the control loops
    omegaController.reset(results.getBestTarget().getYaw());
    xController.reset(robotPose.getX());
    yController.reset(robotPose.getY());


    xController.setGoal(targetPose2d.getX());
    yController.setGoal(targetPose2d.getY());
    omegaController.setGoal(targetPose2d.getRotation().getRadians());

    System.out.println(tgtNum);

    }


  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    
    // pulls an april tag and does the calculations
    vision.updateAprilTagTranslationCMD();


    // lets aprilTag class pass in speed values. These are not scaled - the 
    // scale is in the pVision subsystem
    Pose2d currentRobotPose = vision.currentRobotPose;

    Translation2d difference = 
      new Translation2d(currentRobotPose.getX() - targetPose2d. getX(),
                      currentRobotPose.getY() - targetPose2d.getY());




    // we now have the outputs, next we assign speeds to the swerve system.

    double xspeed = xController.calculate(difference.getX());
    double yspeed = yController.calculate(difference.getY());
    double rotate = omegaController.calculate(currentRobotPose.getRotation().getRadians());


    // // now figure out if at goal

    xAtGoal = xController.atGoal();
    yAtGoal = yController.atGoal();
    rAtGoal = omegaController.atGoal();


    if(xAtGoal) {
      xspeed = 0;
    }
    if(yAtGoal) {
      yspeed = 0;
    }
    if(rAtGoal) {
      rotate = 0;
    }

    if(xController.atGoal() && yController.atGoal() && omegaController.atGoal()) {
      done = true;
    }

    // turn our speeds into an input for the swerve drive modules
    setDriveTranslate = new Translation2d(xspeed, yspeed);
    setDriveRotation = rotate;


    /* Drive  */
    s_Swerve.drive(
      setDriveTranslate,
      setDriveRotation,
      false, // field relitive - we dont care about the field we move to the tag
      true // We will let the PID loop here control the position. We don't want the swerve velocity loop fighting us.
    );


    // TroubleShooting

    SmartDashboard.putNumber("Robot range", difference.getX() );
    SmartDashboard.putNumber("ROBOT strafe",difference.getY());
    SmartDashboard.putNumber("ROBOT ANGLE", currentRobotPose.getRotation().getRadians());

    SmartDashboard.putNumber("XSPEEDCMD", xspeed);
    SmartDashboard.putNumber("YSPEEDCMD", yspeed);
    SmartDashboard.putNumber("OMEGASPEEDCMD", rotate );

    SmartDashboard.putBoolean("X_CONTROLLER_AT_GOAL", xAtGoal);
    SmartDashboard.putBoolean("Y_CONTROLLER_AT_GOAL", yAtGoal);
    SmartDashboard.putBoolean("R_CONTROLLER_AT_GOAL", rAtGoal);

    SmartDashboard.putData(xController);
    SmartDashboard.putData(yController);
    SmartDashboard.putData(omegaController);

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {

    done = false;

  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return (done);
  }
}
