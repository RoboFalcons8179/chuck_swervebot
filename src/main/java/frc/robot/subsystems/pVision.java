package frc.robot.subsystems;


import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonUtils;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class pVision extends SubsystemBase {

    public PhotonCamera camera;
    public PhotonPoseEstimator photonPoseEstimator;

    public double rotDif = 0;
    public double rangeDif = 0;

    public Pose2d currentRobotPose;





    /// create the class

    public pVision() {

        camera = new PhotonCamera("OV5647");
        // camera.setPipelineIndex(1);


    }

    public void updateAprilTagTranslationCMD() {

        // Vision-alignment mode
        // Query the latest result from PhotonVision
        var result = camera.getLatestResult();

        if (result.hasTargets()) {
            // We see a target and need to output our movement commands
            // TODO: MAKE SURE IT IS A VALID TARGET


            // First get range
            rangeDif = PhotonUtils.calculateDistanceToTargetMeters(
                Constants.CamConstants.CAMERA_HEIGHT_METERS,
                Constants.CamConstants.TARGET_HEIGHT_METERS,
                Constants.CamConstants.CAMERA_PITCH_RADIANS,
                Units.degreesToRadians(result.getBestTarget().getPitch()));



            // Then Get rotation
            rotDif = result.getBestTarget().getYaw();

            // turn range and angle into forward and strafe component
            Translation2d movement = new Translation2d(rangeDif, new Rotation2d(Units.degreesToRadians(rotDif)));

            // And make a final pose the robot is at
            currentRobotPose = new Pose2d(movement, new Rotation2d(Units.degreesToRadians(rotDif)));





        } else {

            // no target
            
            rangeDif = 0;
            rotDif = 0;

            currentRobotPose = new Pose2d(0,0, new Rotation2d(0));

        }

        
    }

    public void updateAuto() {



    }

    public void getLimeLight() {

        // add in pulling from the limelight to align with the cones

    }

    // public Translation2d getStrafe() {
    //     // get the strafe vals

    //     // moving towards the april tag - comes from the the range
    //     // moving towards the center line - comes from the angle
    //     Translation2d out = new Translation2d(0,0);









        


    // }

    // public double getRot (Translation2d gyroReading) {

    //     // rotating towards forward. We can grab the gyro val, compare it 
    //     // to the offset, then move there. that assumes we trust the gyro. 
    //     // another option is to just rotate to look directly at the camera.
    //     // That means we trust the camera more.



    //     double out = 0;


    // }

    
}
