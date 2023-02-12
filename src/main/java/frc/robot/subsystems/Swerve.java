package frc.robot.subsystems;

import frc.robot.Constants;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;

import com.kauailabs.navx.frc.AHRS;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.math.filter.LinearFilter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Swerve extends SubsystemBase {
    public SwerveDriveOdometry swerveOdometry;
    public SwerveModule[] mSwerveMods;
    public AHRS gyro;

    // this bool is changed by the fieldToggle function. Should usually be true.
    // public boolean fieldRel;
    private boolean fieldRel;


    // Gyro readings. will pull 10 most recent world readings for X, Y, and Z. 
    // Then does DSP magic (aka averaging) to pull out if the robot is truly still.

    private LinearFilter xAccfilter = LinearFilter.movingAverage(Constants.kBalance.averageWindow);
    private LinearFilter yAccfilter = LinearFilter.movingAverage(Constants.kBalance.averageWindow);
    private LinearFilter zAccfilter = LinearFilter.movingAverage(Constants.kBalance.averageWindow);
    
    double xAcc;
    double yAcc;
    double zAcc;


    public Swerve() {

        
        try {
            System.out.println("--------------");
            gyro = new AHRS(SPI.Port.kMXP); 

            System.out.println("NavX plugged in");
            System.out.println("--------------");


        } catch (RuntimeException ex ) {
            System.out.println("NavX not plugged in");
            System.out.println("--------------");
        }
 
        zeroGyro();

        mSwerveMods = new SwerveModule[] {
            new SwerveModule(0, Constants.Swerve.Mod0.constants),
            new SwerveModule(1, Constants.Swerve.Mod1.constants),
            new SwerveModule(2, Constants.Swerve.Mod2.constants),
            new SwerveModule(3, Constants.Swerve.Mod3.constants)
        };

        /* By pausing init for a second before setting module offsets, we avoid a bug with inverting motors.
         * See https://github.com/Team364/BaseFalconSwerve/issues/8 for more info.
         */
        Timer.delay(1.0);
        resetModulesToAbsolute();

        swerveOdometry = new SwerveDriveOdometry(Constants.Swerve.swerveKinematics, getYaw(), getModulePositions());
    }

    public void drive(Translation2d translation, double rotation, boolean fieldRelative, boolean isOpenLoop) {
        
        // Filter our teleop inputs before we get it. We want to be able to command speeds
        // and positions directly into the function.        
        fieldRel = fieldRelative;
        // Assign and update the swerve
        SwerveModuleState[] swerveModuleStates =
            Constants.Swerve.swerveKinematics.toSwerveModuleStates(
                fieldRelative ?      ChassisSpeeds.fromFieldRelativeSpeeds(
                                    translation.getX(), 
                                    translation.getY(), 
                                    rotation, 
                                    getYaw()
                                )
                                : new ChassisSpeeds(
                                    translation.getX(), 
                                    translation.getY(), 
                                    rotation)
                                );


        SmartDashboard.putNumber("chassis forward", translation.getX());
        SmartDashboard.putNumber("chassis strafe", translation.getY());
        SmartDashboard.putNumber("chassis turn", rotation);

        for(SwerveModule mod : mSwerveMods){
            mod.setDesiredState(swerveModuleStates[mod.moduleNumber], isOpenLoop);
            
        }
    }
    
    

    public void drive_Manually(double speed, Rotation2d angle) {
        // MANUAL MOVEMENT OF BOT. 

        SwerveModuleState[] states = new SwerveModuleState[4];

        for(SwerveModule mod : mSwerveMods){
            
            states[mod.moduleNumber] = new SwerveModuleState(speed, angle);
            mod.setDesiredState(states[mod.moduleNumber], false);
            
        }


    }


    public void driveExtraManual(SwerveModuleState[] states) {
        // probably shouldnt be using this.


        for(SwerveModule mod : mSwerveMods){
            mod.setDesiredState(states[mod.moduleNumber], false);
        }

    }


    /* Used by SwerveControllerCommand in Auto */
    public void setModuleStates(SwerveModuleState[] desiredStates) {
        SwerveDriveKinematics.desaturateWheelSpeeds(desiredStates, Constants.Swerve.maxSpeed);
        
        for(SwerveModule mod : mSwerveMods){
            mod.setDesiredState(desiredStates[mod.moduleNumber], false);
        }
    } 
    
    public void stop() {
        drive( new Translation2d(0,0),0,false,true);
    }

    public Pose2d getPose() {
        return swerveOdometry.getPoseMeters();
    }

    public void resetOdometry(Pose2d pose) {
        swerveOdometry.resetPosition(getYaw(), getModulePositions(), pose);
    }

    public SwerveModuleState[] getModuleStates(){
        SwerveModuleState[] states = new SwerveModuleState[4];
        for(SwerveModule mod : mSwerveMods){
            states[mod.moduleNumber] = mod.getState();
        }
        return states;
    }

    public SwerveModulePosition[] getModulePositions(){
        SwerveModulePosition[] positions = new SwerveModulePosition[4];
        for(SwerveModule mod : mSwerveMods){
            positions[mod.moduleNumber] = mod.getPosition();
        }
        return positions;
    }

    public void zeroGyro(){
        gyro.zeroYaw();
    }

    public Rotation2d getYaw() {
        return (Constants.Swerve.invertGyro) ? Rotation2d.fromDegrees(360 - gyro.getYaw()) : Rotation2d.fromDegrees(gyro.getYaw());
    }

    public void resetModulesToAbsolute(){
        for(SwerveModule mod : mSwerveMods){
            mod.resetToAbsolute();
        }
    }


    public boolean isRobotStill() {

        boolean out = false;

        if (
            xAcc <= Constants.kBalance.StillThreshold &&
            yAcc <= Constants.kBalance.StillThreshold && 
            zAcc <= Constants.kBalance.StillThreshold) {
            out = true;
        }

        return out;
    }

    public boolean isRobotLevel() {
        // remember zAcc is filtered.
        return zAcc <= Constants.kBalance.BalanceAccelThreshold; // ALL ACCELERATION IN G's
    }

    // From Zack's math. finds the angle between gravity and the plane of the Bot, then
    // the angle or the swerve wheels are required to be to point it in the right direction for balencing.
    // In RADs
    public double pointingUpAngle() {

            return Math.acos(gyro.getRawAccelX() / Math.sqrt(Math.pow(gyro.getRawAccelX(), 2) + Math.pow(gyro.getRawAccelY(), 2)));

    }




    @Override
    public void periodic(){
        swerveOdometry.update(getYaw(), getModulePositions());  

        // SWERVE GYRO FILTERING - USED FOR BALENCING
        xAcc = xAccfilter.calculate(gyro.getWorldLinearAccelX());
        yAcc = yAccfilter.calculate(gyro.getWorldLinearAccelY());
        zAcc = zAccfilter.calculate(gyro.getWorldLinearAccelZ());


        /// TROUBLESHOOTING
        Shuffleboard.selectTab("SWERVE");

        for(SwerveModule mod : mSwerveMods){
            SmartDashboard.putNumber("Mod " + mod.moduleNumber + " Cancoder", mod.getCanCoder().getDegrees());
            SmartDashboard.putNumber("Mod " + mod.moduleNumber + " Integrated", mod.getPosition().angle.getDegrees());
            SmartDashboard.putNumber("Mod " + mod.moduleNumber + " Velocity", mod.getState().speedMetersPerSecond);    
        }

        SmartDashboard.putNumber("Overall X", swerveOdometry.getPoseMeters().getX());
        SmartDashboard.putNumber("Overall Y", swerveOdometry.getPoseMeters().getY());
        SmartDashboard.putNumber("Overall Theta", gyro.getYaw());
        SmartDashboard.putBoolean("Field Centric", fieldRel);

        SmartDashboard.putNumber("GYRO X ACCEL", gyro.getRawAccelX());
        SmartDashboard.putNumber("GYRO Y ACCEL", gyro.getRawAccelY());
        SmartDashboard.putNumber("GYRO Z ACCEL", gyro.getRawAccelZ());

        SmartDashboard.putNumber("GYRO X WORLD ACCEL", gyro.getWorldLinearAccelX());
        SmartDashboard.putNumber("GYRO Y WORLD ACCEL", gyro.getWorldLinearAccelY());
        SmartDashboard.putNumber("GYRO Z WORLD ACCEL", gyro.getWorldLinearAccelZ());

        SmartDashboard.putNumber("AccelMag", Math.sqrt(
            Math.pow(gyro.getRawAccelX(), 2) +
            Math.pow(gyro.getRawAccelY(), 2) +
            Math.pow(gyro.getRawAccelZ(), 2)));
  
        SmartDashboard.putNumber("GYRO X FILTER", xAcc);
        SmartDashboard.putNumber("GYRO Y FILTER", yAcc);
        SmartDashboard.putNumber("GYRO Z FILTER", zAcc);

        SmartDashboard.putData(this);

    }


    public double filterInput(double in) {

        return in*in*in;

    }


}