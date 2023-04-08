package frc.robot.commands;

import frc.robot.Constants;
import frc.robot.subsystems.Swerve;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import javax.lang.model.util.ElementScanner14;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;


public class TeleopSwerve extends CommandBase {    

    private Swerve s_Swerve;    

    private DoubleSupplier translationSup;
    private DoubleSupplier strafeSup;
    private DoubleSupplier rotationSup;
    private BooleanSupplier robotCentricSup;
    private BooleanSupplier turbo;
    private BooleanSupplier slowSpeed;

    // Troubleshooting
    private boolean robotCent;
    private boolean robotCenticSupLast;




    public TeleopSwerve(Swerve s_Swerve, DoubleSupplier translationSup, 
        DoubleSupplier strafeSup, DoubleSupplier rotationSup,
        BooleanSupplier robotCentricSup,
        BooleanSupplier turbo, BooleanSupplier slowSpeed
        ) {
        this.s_Swerve = s_Swerve;
        addRequirements(s_Swerve);
        this.turbo = turbo;
        this.translationSup = translationSup;
        this.strafeSup = strafeSup;
        this.rotationSup = rotationSup;
        this.robotCentricSup = robotCentricSup;
        this.slowSpeed = slowSpeed;
        robotCent = false;
        

    }

    @Override
    public void execute() {
        
        
        /* Filter the control values*/
        // double translationVal = s_Swerve.filterInput(MathUtil.applyDeadband(translationSup.getAsDouble(), Constants.stickDeadband));
        // double strafeVal = s_Swerve.filterInput(MathUtil.applyDeadband(strafeSup.getAsDouble(), Constants.stickDeadband));
        // double rotationVal = s_Swerve.filterInput(MathUtil.applyDeadband(rotationSup.getAsDouble(), Constants.stickDeadband));

        double translationVal   = deadband(translationSup.getAsDouble());
        double strafeVal        = deadband(strafeSup.getAsDouble());
        double rotationVal      = deadband(rotationSup.getAsDouble());

        Translation2d init = new Translation2d(translationVal, strafeVal);

        double mag = init.getNorm();

        if (mag > 1)
            mag = 1;
        

        mag = Math.pow(mag, 3);
        

        // // stick filtering
        // translationVal = Math.pow(translationVal, 3);
        // strafeVal = Math.pow(strafeVal, 3);

        // Sam wanted Turbo
         if(turbo.getAsBoolean()==true){

            mag = mag * 1.7;

            // translationVal = 1.5 * translationVal;
            // strafeVal = 1.5 * strafeVal;
            
         } 
        
         // and a brake
         else if(slowSpeed.getAsBoolean()) {
            mag = mag * 0.65;
         }
        
        
         // Toggling Field Relitive
            // IF the last button does not equal this button, and the last mode was false
            // aka when the button is pressed, do this

            // this is overridden by sam for competition to be always on. We can re-enable it when
            // we are done with competition.

        if(robotCentricSup.getAsBoolean() != robotCenticSupLast && robotCenticSupLast == false) {

            robotCent = !robotCent;

        }

        robotCenticSupLast = robotCentricSup.getAsBoolean();

        // Sam request to delete robot centric

        robotCent = false;

        // Initalize Drive Translation 2D values. These are the speeds fed to the controller.
        // These are scaled by the MAX_SPEED and maxAngleVelocity in constants.        
        
//        Translation2d setDriveTranslate = new Translation2d(translationVal, strafeVal).times(Constants.Swerve.maxSpeed);


        Translation2d setDriveTranslate = new Translation2d(mag, init.getAngle()).times(Constants.Swerve.maxSpeed);
        Double setDriveRotation = rotationVal * Constants.Swerve.maxAngularVelocity;
        boolean isOpenLoop = false;

        // go straight forward
        // if (atSpeed.getAsBoolean()) {

        //     translationVal = 0.5;
        //     strafeVal = 0;
        //     rotationVal = 0;
        //     isOpenLoop = false;

            
        //     setDriveTranslate = new Translation2d(translationVal, strafeVal);
        //     setDriveRotation = rotationVal * Constants.Swerve.maxAngularVelocity;

        //     s_Swerve.drive_Manually(setDriveTranslate.getNorm(), setDriveTranslate.getAngle());

        // } else {


    

        /* Drive */
        s_Swerve.drive(
            setDriveTranslate,
            setDriveRotation,
            !robotCent, 
            isOpenLoop
        );
       // }



    }

    private double deadband(double in) {

        if ( in < Constants.stickDeadband && in > -Constants.stickDeadband)
            {return 0;}
        else
            {return in;}


   }
}