// // Copyright (c) FIRST and other WPILib contributors.
// // Open Source Software; you can modify and/or share it under the terms of
// // the WPILib BSD license file in the root directory of this project.

// package frc.robot.commands;

// import org.photonvision.targeting.PhotonPipelineResult;

// import edu.wpi.first.math.geometry.Pose3d;
// import edu.wpi.first.math.geometry.Rotation3d;
// import edu.wpi.first.math.geometry.Transform3d;
// import edu.wpi.first.math.geometry.Translation3d;
// import edu.wpi.first.math.util.Units;
// import edu.wpi.first.wpilibj2.command.CommandBase;
// import frc.robot.Constants;
// import frc.robot.subsystems.Swerve;
// import frc.robot.subsystems.pVision;

// public class goToTagv3 extends CommandBase {
//   /** Creates a new goToTagv3. */

//   Swerve s_Swerve;
//   pVision cam;

//   int targetID;
//   PhotonPipelineResult lastResult;
//   PhotonPipelineResult thisResult;


//   // KNOWN - POSE of the TARGET.
//   Pose3d TARGET = new Pose3d(new Translation3d(0,0,Constants.CamConstants.TARGET_HEIGHT_METERS), 
//   new Rotation3d(0,0,0));


//   // Goal for THE ROBOT to get to. (SPECIFICALLY CENTER FRONT EDGE)
//   Pose3d GOALPOSE = new Pose3d(new Translation3d(Constants.CamConstants.GOAL_RANGE_METERS, 0,0),
//     new Rotation3d(0,0,0));

//   // 3d Transform - measure from front edge to CAM
//   Pose3d ROBOTtoCAM = new Pose3d(new Translation3d(
//     Units.inchesToMeters(-3),
//     Units.inchesToMeters(-6),
//     Units.inchesToMeters(12)),
//     new Rotation3d(
//       0, 
//       Units.degreesToRadians(0),
//       0));

//   // 3 inches from front (neg!)
//   // 6 inches to the right (neg!)
//   // 12 inches up
//   // No Roll/YAW
//   // Some PITCH depending on mounting. This wants RADIANS!!!!!!


//   public goToTagv3(Swerve s_swerve_in, pVision cam_in) {

//     this.s_Swerve = s_swerve_in;
//     this.cam = cam_in;
//     addRequirements(s_Swerve, cam);
//   }

//   // Called when the command is initially scheduled.
//   @Override
//   public void initialize() {

//     thisResult = cam.camera.getLatestResult();


//     if (thisResult.hasTargets()) {
//       // save the target we are looking at
//       targetID = thisResult.getBestTarget().getFiducialId();
//     }
//     else {
      
//       // cancel this command. No Valid targets.
//       System.out.println("PHOTOVISION: NO VALID TARGETS");
//       this.cancel();

//     }

//     // Set the lastResult
//     lastResult = thisResult;
//     thisResult = null;


//   }

//   // Called every time the scheduler runs while the command is scheduled.
//   @Override
//   public void execute() {

//     /// Pull target and get the Transpose we want
//     thisResult = cam.camera.getLatestResult();
    

//     if (!thisResult.hasTargets() || (thisResult.getBestTarget().getFiducialId() != lastResult.getBestTarget().getFiducialId())) {
//       // Cancel this command something went wrong.
//       // Either we lost the target or we are now tracking a different ID
//       System.out.println("Lost Target");
//       this.cancel();
//     }


//     Transform3d camtoTarget = thisResult.getBestTarget().getBestCameraToTarget();
//     // gets the latest result, from camera to target.
    
//     // we want to control so that our transform from our robot to our goal is zero.
    





//     // now we have a diffence. control our way to the target.

//     double xspeed = 0;
//     double yspeed = 0;
//     double ospeed = 0;
    




//     lastResult = thisResult;

//   }

//   // Called once the command ends or is interrupted.
//   @Override
//   public void end(boolean interrupted) {

//     s_Swerve.stop();

//   }

//   // Returns true when the command should end.
//   @Override
//   public boolean isFinished() {
//     return false;
//   }
// }
