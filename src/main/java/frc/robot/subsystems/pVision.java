package frc.robot.subsystems;


import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonUtils;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class pVision extends SubsystemBase {

    public PhotonCamera camera;
    public PhotonPoseEstimator photonPoseEstimator;

    public double rotDif = 0;
    public double rangeDif = 0;

    /// create the class

    public pVision() {

        camera = new PhotonCamera("photonvision");

        // ADD IN DEFINING POSE TRANSLATION STUFF HERE


    }

    public void getAprilTagTranslation() {

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



            // Then Get rot speed
            rotDif = result.getBestTarget().getYaw();

        } else {

            // no target
            
            rangeDif = 0;
            rotDif = 0;

        }


        // Then need to calculate commands. TBD Either send as a auton translation or
        // a speed P loop.

        
    }

    public void getLimeLight() {

        // add in pulling from the limelight to align with the cones

    }



    
}
