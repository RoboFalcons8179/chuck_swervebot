package frc.robot;

import java.util.Map;
import java.util.concurrent.DelayQueue;
import java.util.function.BooleanSupplier;

import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;

import edu.wpi.first.hal.AllianceStationID;
import edu.wpi.first.hal.DriverStationJNI;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj.event.BooleanEvent;
import edu.wpi.first.wpilibj.event.EventLoop;
import edu.wpi.first.wpilibj.event.NetworkBooleanEvent;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import edu.wpi.first.wpilibj2.command.SelectCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.NetworkButton;
import edu.wpi.first.wpilibj2.command.button.POVButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.kBalance;
import frc.robot.autos.*;
import frc.robot.commands.*;
import frc.robot.commands.Limelight.rotateToAngle;
import frc.robot.commands.armStuff.*;
import frc.robot.commands.grabCommands.closeClaw;
import frc.robot.commands.grabCommands.openClaw;
import frc.robot.commands.grabCommands.squeezeClaw;
import frc.robot.subsystems.*;
import frc.robot.commands.BalanceBasic.*;
// import frc.robot.autos.PPTRAJ;
// import frc.robot.autos.traj;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
    /* Controllers */
    private final Joystick driver = new Joystick(0);
    public  final Joystick panel = new Joystick(2);
    private final Joystick board = new Joystick(3);
    
    
    
    
    /* BIG TODO FROM TIM:
     * Clean up the driver buttons and commands. Label them if they are for competition and/or reseved for a function later.
     * YOU ONLY MOVE A BUTTON TO THE RESERVED FOR COMPETION IF THE DRIVE TEAM OR MENTOR APPROVES.
     * 
     * ALSO, print out one or two of these and label it every time you add a function to a button.
     * Do one for the other Controllers too.
     * https://support.xbox.com/en-US/help/hardware-network/controller/xbox-one-wireless-controller
     * 
     * For example, look at the Drive control section below. Add an Arm section and a testing section. Then
     *  use the printout to track buttons.
     */



    /* Drive Control Axis - RESERVED FOR COMPETITION*/
    private final int translationAxis = XboxController.Axis.kLeftY.value;
    private final int strafeAxis = XboxController.Axis.kLeftX.value;
    private final int rotationAxis = XboxController.Axis.kRightX.value;
    private final int rightTrigger = XboxController.Axis.kRightTrigger.value;
    private final int leftTrigger = XboxController.Axis.kLeftTrigger.value;


    /* Driver Buttons and Triggers - RESERVED FOR COMPETITION*/
    private final JoystickButton zeroGyro = new JoystickButton(driver, XboxController.Button.kY.value); // reserved for swerve
        //Y

    private final JoystickButton robotCentric = new JoystickButton(driver, XboxController.Button.kA.value); // reserved for swerve
        // A
    
    private final JoystickButton turbo = new JoystickButton(driver, XboxController.Button.kLeftBumper.value);
        // leftBumper

    /* Extra Driver Remote buttons for testing */
    //private final JoystickButton counterAccel = new JoystickButton(driver, XboxController.Button.kBack.value); // autobalance
    private final JoystickButton driver_x = new JoystickButton(driver, XboxController.Button.kX.value);
    private final JoystickButton driver_b = new JoystickButton(driver, XboxController.Button.kB.value);
    // private final JoystickButton leftDpad = new JoystickButton(driver, getRawAxis(0));
    
    
    private final JoystickButton driver_start = new JoystickButton(driver, XboxController.Button.kStart.value); // reserved for swerve
    private final JoystickButton driver_select = new JoystickButton(driver, XboxController.Button.kBack.value);

    /* ARM BUTTONS/Triggers - Testing */
    private final JoystickButton driver_rb = new JoystickButton(driver, XboxController.Button.kRightBumper.value); // testing for arm

    private final POVButton povUp = new POVButton(driver, 0);
    private final POVButton povRight = new POVButton(driver, 90);
    private final POVButton povDown = new POVButton(driver, 180);
    private final POVButton povLeft = new POVButton(driver, 270);

    /* Arm Axes - COMPERTITION */

    /* ARM Buttons/Triggers - COMPETITION */
    

    // FROM TIM: This is how you do the commmand on the joystick for adjusting the shoulder.
    //      repeat this for the shoulder.

    // From Tim: See this. This is how to do this.
        // shoulderAdjUp.debounce(0.04).onTrue( new {Your Command here})
        // shoulderAdjDown.debounce(0.04).onTrue ( new {Your command here})

    private BooleanSupplier driver_leftTriggerSupplier = new BooleanSupplier() {
        // This part makes the thing that passes up the true false

        @Override
        public boolean getAsBoolean() {

            if(driver.getRawAxis(2) > 0.2)
                return true;
            else
                return false;
        }
        
    };




    private final BooleanSupplier driver_rightTriggerSupplier = new BooleanSupplier() {

        @Override
        public boolean getAsBoolean() {

            if(driver.getRawAxis(3) > 0.2)
                return true;
            else
                return false;
        }
        
    };

    private final Trigger driver_RT = new Trigger(driver_rightTriggerSupplier);


    private final Trigger driver_LT = new Trigger(driver_leftTriggerSupplier);

    // Control Board Stuff//
    private final JoystickButton forwardShoulder = new JoystickButton(board, 1);
    private final JoystickButton backwardShoulder = new JoystickButton(board, 2);
    private final JoystickButton forwardElbow = new JoystickButton(board, 3);
    private final JoystickButton backwardElbow = new JoystickButton(board, 4);
   // private final JoystickButton grabForwardButton1 = new JoystickButton(panel, 1);
    //private final JoystickButton invertSwitchButton1 = new JoystickButton (board, 4);
    //private final JoystickButton invertSwitchButton2 = new JoystickButton (board, 8);
    private final JoystickButton carryButton = new JoystickButton(board, 10);
    private final JoystickButton coneHigh = new JoystickButton(board, 5);
    private final JoystickButton coneMid = new JoystickButton(board, 6);
    private final JoystickButton pickup = new JoystickButton(board, 12);
    private final JoystickButton zero = new JoystickButton(board, 11);
   // private final JoystickButton clawOpenBoard = new JoystickButton(board, 8);
    //private final JoystickButton clawCloseBoard = new JoystickButton(board, 7);
    private final JoystickButton pickupPlayer = new JoystickButton(board, 8);
   
        // Gripper buttons
    private final JoystickButton openClawBoard = new JoystickButton(board, 9);
    private final JoystickButton squeezeClawBoard = new JoystickButton(board, 7);
    private final JoystickButton cubeCarry = new JoystickButton(panel, 4);
    private final JoystickButton cubeSqueeze = new JoystickButton(panel, 6);
    
    /* Subsystems */
    private final Swerve s_Swerve = new Swerve();
    private final Grabber claw = new Grabber();
    public final ArmControl arm = new ArmControl();
    public final Limelight lime = new Limelight();

    /* Troubleshooting, Auto, and Shuffleboard */

    /* Trajectories */
    

   
    

    /** The container for the robot. Contains subsystems, OI devices, and commands. */
    public RobotContainer() {

        // Default Commands

        // swerve will look at the driver for its info
         s_Swerve.setDefaultCommand(
            new TeleopSwerve(
                s_Swerve, 
                () -> -driver.getRawAxis(translationAxis), 
                () -> -driver.getRawAxis(strafeAxis), 
                () -> driver.getRawAxis(rotationAxis), 
                () -> robotCentric.getAsBoolean(),
                () -> turbo.getAsBoolean(),
                () -> driver_b.getAsBoolean()                
            )
        );

        // arm will look at holding the position.
        arm.setDefaultCommand(new defaultArm(arm, board));
        
        // No default for gripper - it will run a command until it is done
        // or interupted.
        // default command for gripper using analog triggers

        // Claw no longer has a defualt command.
        // claw.setDefaultCommand(new updateGripperSpeed(claw, () -> driver.getRawAxis(leftTrigger), () -> driver.getRawAxis(rightTrigger)));

        ///// COMMAND BUTTONS


        // TODO: ORGANIZE THE BUTTONS!!!!!!

        // Set Driver Commands
        doDriverCompetitionCommands();

        doDriverTestCommands();



        // ARM commands

        //doArmCompetitionCommands();

      //  doArmTestCommands();



        // Gripper Test Commands

        doGripperCompetitionCommands();

        doGripperTestCommands();


        // Configure the button bindings
        configureButtonBindings();


        // run the shuffleboard
        runTroubleshooting();
    }



    /* THIS IS NOT FOR TEST CODE. THIS IS ONLY COMMANDS READY FOR COMPETITION.
     * You need to test your code and make sure it will not break the bot.
     * Use this for driver commands only.
     * 
     * TODO: TEST GRIPPER AND ARM CODE TO BE ABLE TO MOVE IT UP TO THE 
     * COMPETITION CODE SECTION
    */

    private void doDriverCompetitionCommands() {

        zeroGyro.onTrue(new InstantCommand(() -> s_Swerve.zeroGyro()));

    }

    /* This is for testing driving commands */
    private void doDriverTestCommands() {

        /// DRIVER BUTTONS

        // open
        driver_LT.whileTrue(new openClaw(claw));

        // close
        driver_RT.onTrue(new closeClaw(claw).withTimeout(1.2));

        // squeeze
        driver_x.onTrue(new squeezeClaw(claw));


        int pov = driver.getPOV(0);

        if (pov == 0){
            System.out.println("test0");
        }

        else if (pov == 180){
            System.out.println("test180");
        }

        /* Driver Buttons */


        // counterAccel.onTrue(new autoBalanceFromInternet(s_Swerve).withTimeout(20));

        // driver go to carry command
        driver_rb.onTrue((new updateHoldPosition(() ->  arm.getHoldShoulder(), () -> 45, arm).repeatedly().alongWith(new squeezeClaw(claw)))
        .until(() -> arm.elbowCurrentAngle() < (60))
        .andThen(new updateHoldPosition(() -> -6, () -> 45, arm)));


        // carry button on control board
        carryButton.onTrue((new updateHoldPosition(() ->  arm.getHoldShoulder(), () -> 45, arm).repeatedly())
        .until(() -> arm.elbowCurrentAngle() < 60)
        .andThen(new updateHoldPosition(() -> -6, () -> 45, arm)));


        // cone high button on control board
        coneHigh.onTrue(new updateHoldPosition(() -> 142, () -> arm.getHoldElbow(), arm).repeatedly()
            .until(() -> arm.shoulderCurrentAngle() > (90))
            .andThen(new updateHoldPosition(() -> 135, () -> 165, arm)));

        coneHigh.onFalse(new updateHoldPosition(() -> 142, () -> arm.getHoldElbow(), arm).repeatedly()
            .until(() -> arm.shoulderCurrentAngle() > (90))
            .andThen(new updateHoldPosition(() -> 128, () -> 165, arm)));    
        

        // cone mid on control board
        coneMid.onTrue(new updateHoldPosition(() -> 115, () -> arm.getHoldElbow(), arm).repeatedly()
            .until(() -> arm.shoulderCurrentAngle() > (65))
            .andThen(new updateHoldPosition(() -> 115, () -> 141, arm)));
            
        coneMid.onFalse(new updateHoldPosition(() -> 115, () -> arm.getHoldElbow(), arm).repeatedly()
            .until(() -> arm.shoulderCurrentAngle() > (65))
            .andThen(new updateHoldPosition(() -> 105, () -> 141, arm)));


        // pickup and score low
        pickup.onTrue(new updateHoldPosition(() -> 45, () -> arm.getHoldElbow(), arm).repeatedly()
                .until(() -> arm.shoulderCurrentAngle() > (30))
                .andThen(new updateHoldPosition(() -> 45, () -> 110, arm)));



        // driver_b.onTrue((new updateHoldPosition(() -> -65, () -> 30, arm).repeatedly())
        //             .until(() -> arm.shoulderCurrentAngle()< (-50))
        //             .andThen(new updateHoldPosition(() -> -65, () -> 120, arm).repeatedly())
        //             .until(() -> arm.elbowCurrentAngle() > 110)
        //             .andThen(new updateHoldPosition(() -> 5, () -> 250, arm))
        //             .andThen(new openClaw(claw).withTimeout(1.2))                     
        //         );


        // driver_b.onFalse(
        //     (new squeezeClaw(claw).withTimeout(.75))
        //     .andThen(new updateHoldPosition(() -> -60, () -> arm.getHoldElbow(), arm).repeatedly())
        //     .until(() -> arm.shoulderCurrentAngle() < (-55))
        //     .andThen(new updateHoldPosition(() ->  arm.getHoldShoulder(), () -> 45, arm).repeatedly())
        //     .until(() -> arm.elbowCurrentAngle() < 60)
        //     .andThen(new updateHoldPosition(() -> -6, () -> 45, arm)
        //     .alongWith(new squeezeClaw(claw)))
        // );



        zero.onTrue((new updateHoldPosition(() ->  arm.getHoldShoulder(), () -> 45, arm).repeatedly())
            .until(() -> arm.elbowCurrentAngle() < (60))
            .andThen(new updateHoldPosition(() -> -6, () -> 45, arm))
            .until(() -> arm.isAtSetpoints())
            .andThen(new updateHoldPosition(() -> -6, () -> 75, arm))
            .withTimeout(1).andThen(new InstantCommand(() -> claw.stop())));
        
        //zero.onTrue(new updateHoldPosition(() -> -6, () -> 75, arm));


        // control board picking up from human player shelf
        pickupPlayer.onTrue((new updateHoldPosition(() -> 115, () -> arm.getHoldElbow(), arm).repeatedly()
        .until(() -> arm.shoulderCurrentAngle() > (90)))
        .andThen(new updateHoldPosition(() -> 125, () -> 177, arm)
            .alongWith(new openClaw(claw).withTimeout(0.7))));
    

        //pickupPlayer.onTrue(new updateHoldPosition(() -> 115, () -> 149, arm));



        // shoulder and elbow adjustments
        forwardShoulder. onTrue(new updateHoldPosition(() -> (arm.getHoldShoulder() + 5), () -> arm.getHoldElbow(), arm));

        backwardShoulder.onTrue(new updateHoldPosition(() -> (arm.getHoldShoulder() - 5), () -> arm.getHoldElbow(), arm));

        forwardElbow.onTrue(new updateHoldPosition(() -> arm.getHoldShoulder(), () -> (arm.getHoldElbow() - 2), arm));

        backwardElbow.onTrue(new updateHoldPosition(() -> arm.getHoldShoulder(), () -> (arm.getHoldElbow() + 2), arm));

        cubeCarry.onTrue((new updateHoldPosition(() ->  arm.getHoldShoulder(), () -> 45, arm).repeatedly())
        .until(() -> arm.elbowCurrentAngle() < 60)
        .andThen(new updateHoldPosition(() -> 25, () -> 38, arm)));

        cubeSqueeze.onTrue((new updateHoldPosition(() ->  arm.getHoldShoulder(), () -> 45, arm).repeatedly())
        .until(() -> arm.elbowCurrentAngle() < 60)
        .andThen(new updateHoldPosition(() -> 25, () -> 60, arm)));

        // Conrol Board Grabber Buttons
        openClawBoard.whileTrue(new openClaw(claw));

        squeezeClawBoard.onTrue(new squeezeClaw(claw));

        povUp.debounce(0.04).whileTrue(new frontUnTip(arm).until(() -> (arm.elbowCurrentAngle() >= 310)));
        povDown.debounce(0.04).whileTrue(new backUnTip(arm).until(() -> (arm.elbowCurrentAngle() <= 50)));

        // Auto driving up to tag stuff
        driver_select.debounce(0.04).whileTrue(
            (new 
            InstantCommand(() -> lime.reflect_init())
            .andThen(new WaitCommand(0.5)))
            .alongWith(new zero(s_Swerve))
            .andThen(new zerolime(s_Swerve, lime)));
        
        driver_start.debounce(0.04).whileTrue(
            (new 
            InstantCommand(() -> lime.april_init())
            .andThen(new WaitCommand(0.5)))
            .alongWith(new zero(s_Swerve))
            .andThen(new zeroTag(s_Swerve, lime)));

        driver_select.onFalse(new 
            InstantCommand(() -> lime.goToDriverCam()));
        driver_start.onFalse(new 
            InstantCommand(() -> lime.goToDriverCam()));



        
        // povLeft.debounce(0.04).whileTrue(
        //     // new InstantCommand(() -> System.out.println(driver.getPOV(pov))));

        // (new 
        // InstantCommand(() -> lime.april_init())
        //     .andThen(new WaitCommand(0.5)))
        //     .alongWith(new zeroTarget(s_Swerve, Rotation2d.fromDegrees(180)))
        //     .andThen(new zeroTagHuman(s_Swerve, lime, 24.0)));

        // povLeft.onFalse(new 
        //     InstantCommand(() -> lime.goToDriverCam()));


        // povRight.debounce(0.04).whileTrue(
        //         // new InstantCommand(() -> System.out.println(driver.getPOV(pov))));
    
        //     (new 
        //     InstantCommand(() -> lime.april_init())
        //         .andThen(new WaitCommand(0.5)))
        //         .alongWith(new zeroTarget(s_Swerve, Rotation2d.fromDegrees(180)))
        //         .andThen(new zeroTagHuman(s_Swerve, lime, -24.0)));
    
        // povRight.onFalse(new 
        //         InstantCommand(() -> lime.goToDriverCam()));



        if (claw.isSqueezing) {

            driver.setRumble(RumbleType.kBothRumble, 1);
        } else {
            driver.setRumble(RumbleType.kBothRumble, 0);
        }
        
    }

    ///// GRABBER

    private void doGripperCompetitionCommands(){



    }

    private void doGripperTestCommands(){


       
       //clawCloseBoard.whileTrue(new closeClaw(claw).withTimeout(Constants.kGrabber.openTimeout));
       //clawOpenBoard.whileTrue(new openClaw(claw).withTimeout(Constants.kGrabber.openTimeout));
       //clawClose.whileTrue(new i(claw).withTimeout(Constants.kGrabbr.openTimeout));

    }


    /**
     * Use this method to define your button->command mappings. Buttons can be created by
     * instantiating a {@link GenericHID} or one of its subclasses ({@link
     * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
     * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
     **/
    private void configureButtonBindings() {
        

  



        // goToTag.debounce(0.04).whileTrue(new chaseTagV2(vision.camera, s_Swerve));
            // This is reseved for later use.

 



        // Arm Troubleshooting



        //Control Board//

        // forwardShoulder.debounce(0.04).whileTrue(new shoulderMove(true));

        // backwardShoulder.debounce(0.04).whileTrue(new shoulderMove(false));

        // forwardElbow.debounce(0.04).whileTrue(new elbowMove(true));

        // backwardElbow.debounce(0.04).whileTrue(new elbowMove(false));

        // invertSwitchButton1.debounce(0.04).whileTrue(new invertSwitch(true));

        // invertSwitchButton2.debounce(0.04).whileTrue(new invertSwitch(false));

        // grabForwardButton1.debounce(0.04).whileTrue(new grabForward(null, claw, true));

        // grabForwardButton2.debounce(0.04).whileTrue(new grabForward(null, claw, false));

        //test.debounce(0.04).whileTrue(new teste());

        // AUTON STUFF

        // LeftS.debounce(0.04).whileTrue(new doPathTrajectory(s_Swerve,LeftSPPTraj).andThen(new doPathTrajectory(s_Swerve, LeftBPPTraj))); // Do the path plan

        // CenterSP.debounce(0.04).whileTrue(new doPathTrajectory(s_Swerve, CenterSPTraj).andThen(new doPathTrajectory(s_Swerve, CenterPSTraj)));

        //CenterSB.debounce(0.04).whileTrue(new doPathTrajectory(s_Swerve, CenterB));

        //RightSPS.debounce(0.04).whileTrue(new doPathTrajectory(s_Swerve, RightS).andThen(new doPathTrajectory(s_Swerve, RightB)));

        // CenterSB.debounce(0.04).whileTrue(new doPathTrajectory(s_Swerve, CenterS).andThen(new doPathTrajectory(s_Swerve, CenterB)));

        // LeftSPS.debounce(0.04).whileTrue(new doPathTrajectory(s_Swerve, LeftSPPTraj).andThen(new doPathTrajectory(s_Swerve, LeftP).andThen(new doPathTrajectory(s_Swerve, LeftPS))));



        
        
    }
    private void runTroubleshooting() {

        SmartDashboard.putData(s_Swerve);
        SmartDashboard.putData(arm);
        SmartDashboard.putData(claw);
    }




    public Command getAutonomousCommand() {

        
        boolean A;
        boolean B;
        boolean C;

        A = panel.getRawButton(3);
        // A:
        //  Blue is on, so if A == true, we are on blue.

        B = panel.getRawButton(2);


        C = panel.getRawButton(1);


        // If one button is on, return X.
        // If aother switch is on, return Y.


        if (A == true) {

            if (!B && !C) {
                return twoferBlueCommand;
            }

            else if ( B && !C) {
                return humpRed;
            }


            else if (!B && C) {
                return basicBalance;
            }

            else if (B && C){
                return spicyBalance;
            }



        } 
        
        else {

            if      (!B && !C) {
                return twoFerRed;
            }

            else if ( B && !C) {
                return humpRed;
            }

            else if (!B && C) {
                return basicBalance;
            }

            else if ( B &&  C){
                return spicyBalance;
            }


        }

        return basicBalance;


        // // IMPORTANT CODE FOR AUTOBALANCE - back up and autobalance.
        // return
        // // Robot Initalization
        // new InstantCommand(()-> s_Swerve.zeroGyro())
        // // .andThen(new InstantCommand(() -> s_Swerve.resetModulesToAbsolute()))

        // // Actual Auton
        // .andThen(new backAndForthCone(s_Swerve, arm, claw))
        // .andThen(new backAndForthCleanup(arm, claw)
        // .alongWith(new autoBalanceFromInternet(s_Swerve, 0.0)));

        // .alongWith(new autoBalanceFromInternet(s_Swerve, 0.0))

        // Go over the hump and autobalance
        // return
        // // Robot Initalization
        // new InstantCommand(()-> s_Swerve.zeroGyro())
        // // .andThen(new InstantCommand(() -> s_Swerve.resetModulesToAbsolute()))

        // // Actual Auton
        // .andThen(new backAndForthCone(s_Swerve, arm, claw))
        // .andThen( (new backAndForthCleanup(arm, claw))
        // .alongWith(new TeleopSwerve(
        //     s_Swerve, 
        //     () -> 0.915, //.93, 
        //     () -> 0, 
        //     () -> 0, 
        //     () -> true,
        //     () -> true                
        // )).withTimeout(3.20))

        
        
        // .andThen(new zero(s_Swerve))
        // .andThen(new autoBalanceFromInternet(s_Swerve, Math.PI));



        // return 

        // new InstantCommand(()-> s_Swerve.zeroGyro())
        // .andThen(new backAndForthCone(s_Swerve, arm, claw))
        // .andThen(new backAndForthCleanup(arm, claw)
        // .alongWith(new doPathTrajectory(s_Swerve, PPTRAJ.StraightR)))
        // // .alongWith(new doTrajectory(s_Swerve, traj.wireBump)))

        // .andThen((new updateHoldPosition(() -> 45, () -> arm.elbowSetpoint, arm).repeatedly()
        //     .alongWith(new openClaw(claw)))
        //     // .withTimeout(0.1)
        //     .until(() -> arm.shoulderCurrentAngle() > (25))
        //     .andThen(new updateHoldPosition(() -> 45, () -> 110, arm)))
        // .andThen(new squeezeClaw(claw).withTimeout(1.0))
        // // should have the cube by now

        // .andThen((new updateHoldPosition(() ->  arm.getHoldShoulder(), () -> 45, arm).repeatedly().alongWith(new squeezeClaw(claw)))
        // .until(() -> arm.elbowCurrentAngle() < 60)
        // .andThen(new updateHoldPosition(() -> -6, () -> 45, arm)))
        // // have the cube

        // .andThen(new 
        // InstantCommand(() -> lime.april_init())
        // .andThen(new WaitCommand(0.5))
        // .alongWith(new zero(s_Swerve)))
        // .andThen((new InstantCommand(() -> 
        
        //     s_Swerve.drive(new Translation2d(-2,0), 
        //     0,
        //     true,
        //     true)
            
        //     ).repeatedly()).withTimeout(0.8))
        
        //     //.andThen(new zero(s_Swerve))
        // .andThen(new zeroTag(s_Swerve, lime))

        // // put the cube on the thing

        // // .andThen(new updateHoldPosition(() -> 115, () -> arm.getHoldElbow(), arm).repeatedly()
        // //     .until(() -> arm.shoulderCurrentAngle() > (90))
        // // .andThen(new updateHoldPosition(() -> 115, () -> 141, arm).repeatedly())
        // //     .until(() -> arm.isAtSetpoints()))
        

        // .alongWith((new updateHoldPosition(() -> 45, () -> arm.getHoldElbow(), arm).repeatedly()
        //     .until(() -> arm.shoulderCurrentAngle() > (30))
        //     .andThen(new updateHoldPosition(() -> 45, () -> 110, arm))).until(() -> arm.isAtSetpoints()))

        // // .andThen(new openClaw(claw).withTimeout(1))

        // // .andThen(new closeClaw(claw))

        // // .andThen(((new updateHoldPosition(() ->  arm.getHoldShoulder(), () -> 45, arm).repeatedly())
        // //     .alongWith(new squeezeClaw(claw)))
        // // .until(() -> arm.elbowCurrentAngle() < 60))
        // // .andThen(new updateHoldPosition(() -> -6, () -> 45, arm));


        
        



        // From Sam's testing
        // .andThen(new doPathTrajectory(s_Swerve, PPTRAJ.balance15M1)
        //         .alongWith(new backAndForthCleanup(arm, claw)
        //                     .andThen(new updateHoldPosition(() -> -6, () -> 45, arm)))
        
        
        //                     );

        //.andThen(new grabCube(s_Swerve, arm, claw))
        //.andThen(new doTrajectory(s_Swerve, balance15M2))
       //.andThen(new backAndForth(s_Swerve, arm, claw));
//right     

// return new backAndForth(s_Swerve, arm, claw);

// return new doPathTrajectory(s_Swerve, CenterPSTraj);

 //left
    /*   
    return new backAndForth(s_Swerve, arm, claw).andThen( new InstantCommand(()-> s_Swerve.zeroGyro())).andThen(new InstantCommand(() -> s_Swerve.resetModulesToAbsolute())).andThen(new TeleopSwerve(
    s_Swerve, 
    () -> 0, 
    () -> .75, 
    () -> 0, 
    () -> true,
    () -> goSpeed.getAsBoolean()                
)).withTimeout(.5)
.andThen(new TeleopSwerve(
    s_Swerve, 
    () -> 1, 
    () -> 0, 
    () -> 0, 
    () -> true,
    () -> goSpeed.getAsBoolean()                
)).withTimeout(2);
    */
          // return new doPathTrajectory(s_Swerve,compMiddleBlue);
        //centerBlue 
    //.andThen(new WaitCommand(0.75)).andThen(new doPathTrajectory(s_Swerve, compMiddleBlue));

        //center Red
      //return new backAndForth(s_Swerve, arm, claw).andThen(new doPathTrajectory(s_Swerve, compMiddleRed));

        //right blue 
        //return new backAndForth(s_Swerve, arm, claw).andThen(new doPathTrajectory(s_Swerve, compRightBlue));

        //right red
      // return new backAndForth(s_Swerve, arm, claw).andThen(new doPathTrajectory(s_Swerve, compRightRed));
        
       //left blue
      // return new backAndForth(s_Swerve, arm, claw).andThen(new doPathTrajectory(s_Swerve, compLeftBlue));

       //left red
     // return new backAndForth(s_Swerve, arm, claw).andThen(new doPathTrajectory(s_Swerve, compLeftRed));
     



        /*if (test.getAsBoolean() == true){
           return new doPathTrajectory(s_Swerve, CenterS);
        }

        else{
            return (new doPathTrajectory(s_Swerve, CenterB));
        }*/



    }
    public Swerve getSwervePointer(){
        return s_Swerve;
    }


     /* AUTO COMMANDS */

     public Command basicBalance = 
     // Robot Initalization
     new InstantCommand(()-> s_Swerve.zeroGyro())
     // .andThen(new InstantCommand(() -> s_Swerve.resetModulesToAbsolute()))

     // Actual Auton
     .andThen(new backAndForth(s_Swerve, arm, claw))
     .andThen(new backAndForthCleanup(arm, claw)
     .alongWith(new autoBalanceFromInternet(s_Swerve, 0.0)));

     // .andThen(new InstantCommand(() -> s_Swerve.drive_Manually(0.05, new Rotation2d(Math.toRadians(90)))).withTimeout(0.1))
     // .andThen(new InstantCommand(() -> s_Swerve.stop()));


 public Command spicyBalance = 

     new InstantCommand(()-> s_Swerve.zeroGyro())
     // .andThen(new InstantCommand(() -> s_Swerve.resetModulesToAbsolute()))

     // Actual Auton
     .andThen(new backAndForth(s_Swerve, arm, claw))


     .andThen( 
        
        // (new backAndForthCleanup(arm, claw))

        ////

        (new closeClaw(claw).withTimeout(0.5))
        // new closeClaw(claw).withTimeout(0.5)
        
        .alongWith(new updateHoldPosition(() -> -6, () -> 45, arm))

        /////



        .alongWith(new TeleopSwerve(
            s_Swerve, 
            () -> 0.75, //.915
            () -> 0, 
            () -> 0, 
            () -> false,
            () -> true,
            () -> false                
        )).withTimeout(3)
        
    )//3.2

     
     .andThen(new InstantCommand(() -> claw.stop()))
     .andThen(new zero(s_Swerve))
     .andThen(new autoBalanceFromInternet(s_Swerve, Math.PI));
    




 public Command twoFerRed =
 (
 ((new InstantCommand(()-> s_Swerve.zeroGyro()))
 
 // score first cone
 .andThen(new backAndForthCone(s_Swerve, arm, claw))
 .andThen(new backAndForthCleanup(arm, claw)

 // move to next piece
     .alongWith(new doTrajectory(s_Swerve, traj.rotate)))

 

 // pickup next piece
 .andThen((new updateHoldPosition(() -> 45, () -> arm.elbowSetpoint, arm).repeatedly()
     .alongWith(new openClaw(claw)))
     // .withTimeout(0.1)
     .until(() -> arm.shoulderCurrentAngle() > (25))
     .andThen(new updateHoldPosition(() -> 45, () -> 110, arm)))
 
 .andThen(new squeezeClaw(claw).withTimeout(1.0))

 // Rotate back, initalize the camera, and bring arm back to carry

 
 // .andThen((new zero(s_Swerve))
 //     .alongWith( 
 //         ((new updateHoldPosition(() ->  arm.getHoldShoulder(), () -> 45, arm).repeatedly().alongWith(new squeezeClaw(claw)))
 //         .until(() -> arm.elbowCurrentAngle() < (60))
 //         .andThen(new updateHoldPosition(() -> -6, () -> 45, arm))))



// Either use above or below block. Use beloe for going to the april tag.

 .andThen( new InstantCommand(() -> lime.april_init())
    //  .alongWith(new zero(s_Swerve))
     .alongWith( 
         ((new updateHoldPosition(() ->  arm.getHoldShoulder(), () -> 45, arm).repeatedly().alongWith(new closeClaw(claw)))
         .until(() -> arm.elbowCurrentAngle() < (60))
         .andThen(new updateHoldPosition(() -> -6, () -> 45, arm))))))

 );
 
 

//  // zoom forward
//  .andThen(((new InstantCommand(() -> 

//      s_Swerve.drive(new Translation2d(-6,0.2), 
//          0,
//          true,
//          false)
     
//      )).repeatedly()).withTimeout(0.9))



 // .andThen(new zeroTag(s_Swerve, lime))



 public Command twoferBlueCommand = 

 (
    new InstantCommand(()-> s_Swerve.zeroGyro())
 
    // score first cone
    .andThen(new backAndForthCone(s_Swerve, arm, claw))
    
   
    // move to next piece
        .andThen((new doTrajectory(s_Swerve, traj.humpRed))
        .alongWith((new squeezeClaw(claw).withTimeout(0.1))
        .andThen( new updateHoldPosition(() -> -6, () -> 45, arm).repeatedly())
        .until(() -> arm.shoulderCurrentAngle()< (0))
        .andThen( new updateHoldPosition(() -> -65, () -> 30, arm).repeatedly())
        .until(() -> arm.shoulderCurrentAngle()< (-60))
       ))
       .andThen((new TeleopSwerve(
           s_Swerve, 
           () -> 0, 
           () -> 0, 
           () -> 0, 
           () -> true,
           () -> true,
           () -> false               
       )).withTimeout(.1)
                   .andThen(new updateHoldPosition(() -> -65, () -> 120, arm).repeatedly())
                   .until(() -> arm.elbowCurrentAngle() > 110)
                   .andThen(new updateHoldPosition(() -> 5, () -> 250, arm))   
                   .andThen(new openClaw(claw).withTimeout(1.2))
                   )
       .andThen(new squeezeClaw(claw).withTimeout(.75)
               .andThen(new updateHoldPosition(() -> -60, () -> arm.getHoldElbow(), arm).repeatedly())
               .until(() -> arm.shoulderCurrentAngle() < (-55))
               .andThen(new updateHoldPosition(() ->  arm.getHoldShoulder(), () -> 45, arm).repeatedly())
               .until(() -> arm.elbowCurrentAngle() < 60)
               .andThen(new updateHoldPosition(() -> -6, () -> 45, arm))
               )
   
//      ((new InstantCommand(()-> s_Swerve.zeroGyro()))
     
//      // score first cone
//      .andThen(new backAndForthCone(s_Swerve, arm, claw))
     
//      .andThen(new backAndForthCleanup(arm, claw)

//      // move to next piece
//          .alongWith(new doTrajectory(s_Swerve, traj.rotateBlue))))


//      .andThen(new zeroTarget(s_Swerve, Rotation2d.fromDegrees(-158)))

//      // pickup next piece
//      .andThen((new updateHoldPosition(() -> 45, () -> arm.elbowSetpoint, arm).repeatedly()
//          .alongWith(new openClaw(claw)))
//          // .withTimeout(0.1)
//          .until(() -> arm.shoulderCurrentAngle() > (25))
//          .andThen(new updateHoldPosition(() -> 45, () -> 110, arm)))
//      .andThen(new squeezeClaw(claw).withTimeout(1.0))

//      // Rotate back, initalize the camera, and bring arm back to carry
 
     
//      // .andThen((new zero(s_Swerve))
//      //     .alongWith( 
//      //         ((new updateHoldPosition(() ->  arm.getHoldShoulder(), () -> 45, arm).repeatedly().alongWith(new squeezeClaw(claw)))
//      //         .until(() -> arm.elbowCurrentAngle() < (60))
//      //         .andThen(new updateHoldPosition(() -> -6, () -> 45, arm))))

//      )

//  // Either use above or below block. Use beloe for going to the april tag.

//      .andThen( new InstantCommand(() -> lime.april_init())
//         //  .alongWith(new zero(s_Swerve))
//          .alongWith( 
//              ((new updateHoldPosition(() ->  arm.getHoldShoulder(), () -> 45, arm).repeatedly().alongWith(new squeezeClaw(claw)))
//              .until(() -> arm.elbowCurrentAngle() < (60))
//              .andThen(new updateHoldPosition(() -> -6, () -> 45, arm))))

     // )
     
     
     )

     // zoom forward
    //  .andThen(((new InstantCommand(() -> 
 
    //      s_Swerve.drive(new Translation2d(-6,-0.2), 
    //          0,
    //          true,
    //          false)
         
    //      )).repeatedly()).withTimeout(0.9))

         ;


 public Command humpRed = 
 new InstantCommand(()-> s_Swerve.zeroGyro())
 
 // score first cone
 .andThen(new backAndForthCone(s_Swerve, arm, claw))
 

 // move to next piece
     .andThen((new doTrajectory(s_Swerve, traj.humpRed))
     .alongWith((new squeezeClaw(claw).withTimeout(0.1))
     .andThen( new updateHoldPosition(() -> -6, () -> 45, arm).repeatedly())
     .until(() -> arm.shoulderCurrentAngle()< (0))
     .andThen( new updateHoldPosition(() -> -65, () -> 30, arm).repeatedly())
     .until(() -> arm.shoulderCurrentAngle()< (-60))
    ))
    .andThen((new TeleopSwerve(
        s_Swerve, 
        () -> 0, 
        () -> 0, 
        () -> 0, 
        () -> true,
        () -> true,
        () -> false               
    )).withTimeout(.1)
                .andThen(new updateHoldPosition(() -> -65, () -> 120, arm).repeatedly())
                .until(() -> arm.elbowCurrentAngle() > 110)
                .andThen(new updateHoldPosition(() -> 5, () -> 250, arm))   
                .andThen(new openClaw(claw).withTimeout(1.2))
                )
    .andThen(new squeezeClaw(claw).withTimeout(.75)
            .andThen(new updateHoldPosition(() -> -60, () -> arm.getHoldElbow(), arm).repeatedly())
            .until(() -> arm.shoulderCurrentAngle() < (-55))
            .andThen(new updateHoldPosition(() ->  arm.getHoldShoulder(), () -> 45, arm).repeatedly())
            .until(() -> arm.elbowCurrentAngle() < 60)
            .andThen(new updateHoldPosition(() -> -6, () -> 45, arm))
            )

            //sams attempt at score the grabbed cube
    //         .andThen (new zero(s_Swerve)
    //         .alongWith(new InstantCommand(() -> lime.april_init())))
    //         .andThen(new backAndForth(s_Swerve, arm, claw))


    //  .andThen( 

    //     (new closeClaw(claw).withTimeout(0.5))
        
    //     .alongWith(new updateHoldPosition(() -> -6, () -> 45, arm))
    
    //         )
              
                
                ;
                


             

 



//  //.andThen(new zeroTarget(s_Swerve, Rotation2d.fromDegrees(-170)))
// //old code
// //  // pickup next piece
// //  .andThen((new updateHoldPosition(() -> 45, () -> arm.elbowSetpoint, arm).repeatedly()
// //      .alongWith(new openClaw(claw)))
// //      // .withTimeout(0.1)
// //      .until(() -> arm.shoulderCurrentAngle() > (25))
// //      .andThen(new updateHoldPosition(() -> 45, () -> 110, arm)))
// //  .andThen(new squeezeClaw(claw).withTimeout(1.0))

//  // Rotate back, initalize the camera, and bring arm back to carry

 
//  // .andThen((new zero(s_Swerve))
//  //     .alongWith( 
//  //         ((new updateHoldPosition(() ->  arm.getHoldShoulder(), () -> 45, arm).repeatedly().alongWith(new squeezeClaw(claw)))
//  //         .until(() -> arm.elbowCurrentAngle() < (60))
//  //         .andThen(new updateHoldPosition(() -> -6, () -> 45, arm))))

 

// // Either use above or below block. Use beloe for going to the april tag.

//  .andThen( new InstantCommand(() -> lime.april_init())
//     //  .alongWith(new zero(s_Swerve))
//      .alongWith( 
//          ((new updateHoldPosition(() ->  arm.getHoldShoulder(), () -> 45, arm).repeatedly().alongWith(new squeezeClaw(claw)))
//          .until(() -> arm.elbowCurrentAngle() < (60))
//          .andThen(new updateHoldPosition(() -> -6, () -> 45, arm))))

//  )

//    ;
 // )
 
 
 
//  // zoom forward
//  .andThen(((new InstantCommand(() -> 

//      s_Swerve.drive(new Translation2d(-3,0), 
//          0,
//          true,
//          false)
     
//      )).repeatedly()).withTimeout(1.05))

 // .andThen(new zeroTag(s_Swerve, lime))
 public Command humpBlue = 
 (
 ((new InstantCommand(()-> s_Swerve.zeroGyro()))
 
 // score first cone
 .andThen(new backAndForthCone(s_Swerve, arm, claw))
 .andThen(new backAndForthCleanup(arm, claw)

 // move to next piece
     .alongWith(new doTrajectory(s_Swerve, traj.humpBlue))))


 .andThen(new zeroTarget(s_Swerve, Rotation2d.fromDegrees(-175)))

 // pickup next piece
 .andThen((new updateHoldPosition(() -> 45, () -> arm.elbowSetpoint, arm).repeatedly()
     .alongWith(new openClaw(claw)))
     // .withTimeout(0.1)
     .until(() -> arm.shoulderCurrentAngle() > (25))
     .andThen(new updateHoldPosition(() -> 45, () -> 110, arm)))
 .andThen(new squeezeClaw(claw).withTimeout(1.0))

 // Rotate back, initalize the camera, and bring arm back to carry

 
 // .andThen((new zero(s_Swerve))
 //     .alongWith( 
 //         ((new updateHoldPosition(() ->  arm.getHoldShoulder(), () -> 45, arm).repeatedly().alongWith(new squeezeClaw(claw)))
 //         .until(() -> arm.elbowCurrentAngle() < (60))
 //         .andThen(new updateHoldPosition(() -> -6, () -> 45, arm))))

 )

// Either use above or below block. Use beloe for going to the april tag.

 .andThen( new InstantCommand(() -> lime.april_init())
    //  .alongWith(new zero(s_Swerve))
     .alongWith( 
         ((new updateHoldPosition(() ->  arm.getHoldShoulder(), () -> 45, arm).repeatedly().alongWith(new squeezeClaw(claw)))
         .until(() -> arm.elbowCurrentAngle() < (60))
         .andThen(new updateHoldPosition(() -> -6, () -> 45, arm))))

 // )
 
 
 )

 // zoom forward
//  .andThen(((new InstantCommand(() -> 

//      s_Swerve.drive(new Translation2d(-3,0), 
//          0,
//          true,
//          false)
     
//      )).repeatedly()).withTimeout(0.9))

 // .andThen(new zeroTag(s_Swerve, lime))
 
 
 
 ;

    

}
