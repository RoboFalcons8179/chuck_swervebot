package frc.robot;

import java.util.Map;
import java.util.concurrent.DelayQueue;
import java.util.function.BooleanSupplier;

import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;

import edu.wpi.first.hal.AllianceStationID;
import edu.wpi.first.hal.DriverStationJNI;
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
import frc.robot.commands.BalanceBasic.*;;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
    /* Controllers */
    private final Joystick driver = new Joystick(0);
    private final Joystick panel = new Joystick(2);
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
    
    
    private final JoystickButton driver_start = new JoystickButton(driver, XboxController.Button.kStart.value); // reserved for swerve
    private final JoystickButton driver_select = new JoystickButton(driver, XboxController.Button.kBack.value);

    /* ARM BUTTONS/Triggers - Testing */
    private final JoystickButton driver_rb = new JoystickButton(driver, XboxController.Button.kRightBumper.value); // testing for arm

    

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

    private final Trigger driver_LT = new Trigger(driver_leftTriggerSupplier);
        // This part makes it work with the command archetecture.


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


    // private final BooleanSupplier isShoudlerAdjustUp = new BooleanSupplier() {
    //     // This part makes the thing that passes up the true false

    //     @Override
    //     public boolean getAsBoolean() {

    //         if(panel.getRawAxis(panelY) == -1)
    //             return true;
    //         else
    //             return false;
    //     }
        
    // };

    // private final Trigger shoulderAdjUp = new Trigger(isShoudlerAdjustUp);
    //     // This part makes it work with the command archetecture.


    // private final BooleanSupplier isShoulderAdjustDown = new BooleanSupplier() {

    //     @Override
    //     public boolean getAsBoolean() {

    //         if(panel.getRawAxis(panelY) == 1)
    //             return true;
    //         else
    //             return false;
    //     }
        
    // };

    // private final Trigger shoulderAdjDown = new Trigger(isShoulderAdjustDown);


    //private final Joystick manualShoulder = new JoystickButton(panel, rotationAxis);
    
    // private final JoystickButton test = new JoystickButton(driver, XboxController.Button.kB.value);
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



    // When we get a new switch change button number to what the switch is//
    //private final JoystickButton grabForwardButton2 = new JoystickButton(panel, 0);

    
    // Stick buttons
    // private final JoystickButton LeftSPS = new JoystickButton(stick, 9);
    // private final JoystickButton LeftS = new JoystickButton(stick, 11);
    // private final JoystickButton CenterSP = new JoystickButton(stick, 12);
    // private final JoystickButton CenterSB = new JoystickButton(stick, 10);
    //private final JoystickButton RightSPS = new JoystickButton(stick, 10);
    //private final JoystickButton CenterSB = new JoystickButton(stick, 9);

    // Switch buttons
     private final JoystickButton test = new JoystickButton(panel, 1);
     //private final JoystickButton test2 = new JoystickButton(panel, 0);

    /* Subsystems */
    private final Swerve s_Swerve = new Swerve();
    private final Grabber claw = new Grabber();
    public final ArmControl arm = new ArmControl();
    public final Limelight lime = new Limelight();

    /* Troubleshooting, Auto, and Shuffleboard */


    


    // public BooleanEvent povUp(EventLoop loop){
    //     return null;
    // }



   



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
                () -> turbo.getAsBoolean()                
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
        carryButton.onTrue((new updateHoldPosition(() ->  arm.getHoldShoulder(), () -> 45, arm).repeatedly().alongWith(new squeezeClaw(claw)))
        .until(() -> arm.elbowCurrentAngle() < 60)
        .andThen(new updateHoldPosition(() -> -6, () -> 45, arm)));


        // cone high button on control board
        coneHigh.onTrue(new updateHoldPosition(() -> 142, () -> arm.getHoldElbow(), arm).repeatedly()
            .until(() -> arm.shoulderCurrentAngle() > (90))
            .andThen(new updateHoldPosition(() -> 142, () -> 165, arm)));
        

        // cone mid on control board
        coneMid.onTrue(new updateHoldPosition(() -> 115, () -> arm.getHoldElbow(), arm).repeatedly()
            .until(() -> arm.shoulderCurrentAngle() > (90))
            .andThen(new updateHoldPosition(() -> 115, () -> 141, arm)));


        // pickup and score low
        pickup.onTrue(new updateHoldPosition(() -> 45, () -> arm.getHoldElbow(), arm).repeatedly()
                .until(() -> arm.shoulderCurrentAngle() > (30))
                .andThen(new updateHoldPosition(() -> 45, () -> 110, arm)));

        // pickup.onTrue(new updateHoldPosition(() -> 45, () -> 118, arm));

        //pickupAbove.onTrue(new updateHoldPosition(() -> 100, () -> 225, arm));

        zero.onTrue((new updateHoldPosition(() ->  arm.getHoldShoulder(), () -> 45, arm).repeatedly().alongWith(new squeezeClaw(claw)))
            .until(() -> arm.elbowCurrentAngle() < (60))
            .andThen(new updateHoldPosition(() -> -6, () -> 45, arm))
            .until(() -> arm.isAtSetpoints())
            .andThen(new updateHoldPosition(() -> -6, () -> 75, arm))
            .withTimeout(1).andThen(new InstantCommand(() -> claw.stop())));
        
        //zero.onTrue(new updateHoldPosition(() -> -6, () -> 75, arm));


        // control board picking up from human player shelf
        pickupPlayer.onTrue(new updateHoldPosition(() -> 115, () -> arm.getHoldElbow(), arm).repeatedly()
        .until(() -> arm.shoulderCurrentAngle() > (90))
        .andThen(new updateHoldPosition(() -> 115, () -> 149, arm)
            .alongWith(new openClaw(claw)).withTimeout(1.2)));
    

        //pickupPlayer.onTrue(new updateHoldPosition(() -> 115, () -> 149, arm));



        // shoulder and elbow adjustments
        forwardShoulder.onTrue(new updateHoldPosition(() -> (arm.getHoldShoulder() + 5), () -> arm.getHoldElbow(), arm));

        backwardShoulder.onTrue(new updateHoldPosition(() -> (arm.getHoldShoulder() - 5), () -> arm.getHoldElbow(), arm));

        forwardElbow.onTrue(new updateHoldPosition(() -> arm.getHoldShoulder(), () -> (arm.getHoldElbow() + 2), arm));

        backwardElbow.onTrue(new updateHoldPosition(() -> arm.getHoldShoulder(), () -> (arm.getHoldElbow() - 2), arm));





        // Conrol Board Grabber Buttons
        openClawBoard.whileTrue(new openClaw(claw));

        squeezeClawBoard.onTrue(new squeezeClaw(claw));


        // Auto driving stuff
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




        if (claw.grabberMotor.getStatorCurrent() > 2) {

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

        Shuffleboard.selectTab("MAIN");
        SmartDashboard.putData(s_Swerve);
        SmartDashboard.putData(arm);
        SmartDashboard.putData(claw);
        
    }




/*public static void getAlliance() {
             
    AllianceStationID allianceID = DriverStationJNI.getAllianceStation();
    String test = "test";
    //numbers 1-3 are for red and numbers 4-6 are for blue
    //waiting for Tim
    boolean isRedAlliance = false;
    int stationNumber = 1;
    switch (allianceID) {                 
        case Red1:                     
            isRedAlliance = true;
            stationNumber = 1;
            System.out.println(test);
            break;
        case Red2:                    
            isRedAlliance = true;
            stationNumber = 2;
            break;
        case Red3:                     
            isRedAlliance = true;
            stationNumber = 3;
            break;
        case Blue1:                     
            isRedAlliance = false;
            stationNumber = 4;
            break;
        case Blue2:                     
            isRedAlliance = false;
            stationNumber = 5;
            break;
        case Blue3:                     
            isRedAlliance = false;
            stationNumber = 6;
            break;
          default:
            //return Alliance.Invalid;
        }
      }*/


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

        return
        // Robot Initalization
        new InstantCommand(()-> s_Swerve.zeroGyro())
        .andThen(new InstantCommand(() -> s_Swerve.resetModulesToAbsolute()))

        // Actual Auton
        .andThen(new backAndForthCone(s_Swerve, arm, claw))
        .andThen(new backAndForthCleanup(arm, claw)
        .alongWith(new autoBalanceFromInternet(s_Swerve)));

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
}
