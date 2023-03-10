package frc.robot;

import java.util.Map;
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
import frc.robot.autos.*;
import frc.robot.commands.*;
import frc.robot.commands.armStuff.defaultArm;
import frc.robot.commands.armStuff.goto30;
import frc.robot.commands.armStuff.gotoArmGeneralLocation;
import frc.robot.subsystems.*;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
    /* Controllers */
    private final Joystick driver = new Joystick(0);
    private final Joystick stick = new Joystick(1);
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


    /* Driver Buttons and Triggers - RESERVED FOR COMPETITION*/
    private final JoystickButton zeroGyro = new JoystickButton(driver, XboxController.Button.kY.value); // reserved for swerve
    private final JoystickButton robotCentric = new JoystickButton(driver, XboxController.Button.kLeftBumper.value); // reserved for swerve
    private final JoystickButton goSpeed = new JoystickButton(driver, XboxController.Button.kStart.value); // reserved for swerve



    /* Extra Driver Remote buttons for testing */

    private final JoystickButton counterAccel = new JoystickButton(driver, XboxController.Button.kBack.value); // autobalance
    private final JoystickButton holdBot = new JoystickButton(driver, XboxController.Button.kA.value); // currently for claw.
    private final JoystickButton clawOpen = new JoystickButton(driver, XboxController.Button.kX.value);

    
    
    

    /* Arm Axes - COMPERTITION */

    /* ARM Buttons/Triggers - COMPETITION */
    
    /* ARM AXES - Testing */
    private final int panelX = Joystick.kDefaultXChannel;
    private final int panelY = Joystick.kDefaultYChannel;

    /* ARM BUTTONS/Triggers - Testing */
    private final JoystickButton slowForward = new JoystickButton(driver, XboxController.Button.kRightBumper.value); // testing for arm

    // FROM TIM: This is how you do the commmand on the joystick for adjusting the shoulder.
    //      repeat this for the shoulder.

    private final BooleanSupplier isArmAdjustUp = new BooleanSupplier() {
        // This part makes the thing that passes up the true false

        @Override
        public boolean getAsBoolean() {

            if(panel.getRawAxis(panelX) == 1)
                return true;
            else
                return false;
        }
        
    };

    private final Trigger shoulderAdjUp = new Trigger(isArmAdjustUp);
        // This part makes it work with the command archetecture.


    private final BooleanSupplier isArmAdjustDown = new BooleanSupplier() {

        @Override
        public boolean getAsBoolean() {

            if(panel.getRawAxis(panelX) == -1)
                return true;
            else
                return false;
        }
        
    };

    private final Trigger shoulderAdjDown = new Trigger(isArmAdjustDown);


    //private final Joystick manualShoulder = new JoystickButton(panel, rotationAxis);
    
    // private final JoystickButton test = new JoystickButton(driver, XboxController.Button.kB.value);
    // Control Board Stuff//
    private final JoystickButton forwardShoulder = new JoystickButton(board, 2);
    private final JoystickButton backwardShoulder = new JoystickButton(board, 1);
    private final JoystickButton forwardElbow = new JoystickButton(board, 7);
    private final JoystickButton backwardElbow = new JoystickButton(board, 3);
    private final JoystickButton grabForwardButton1 = new JoystickButton(panel, 1);
    private final JoystickButton invertSwitchButton1 = new JoystickButton (board, 4);
    private final JoystickButton invertSwitchButton2 = new JoystickButton (board, 8);
    // When we get a new switch change button number to what the switch is//
    private final JoystickButton grabForwardButton2 = new JoystickButton(panel, 0);

    
    // Stick buttons
    private final JoystickButton LeftSPS = new JoystickButton(stick, 9);
    private final JoystickButton LeftS = new JoystickButton(stick, 11);
    private final JoystickButton CenterSP = new JoystickButton(stick, 12);
    private final JoystickButton CenterSB = new JoystickButton(stick, 10);
    //private final JoystickButton RightSPS = new JoystickButton(stick, 10);
    //private final JoystickButton CenterSB = new JoystickButton(stick, 9);

    // Switch buttons
     private final JoystickButton test = new JoystickButton(panel, 1);
     //private final JoystickButton test2 = new JoystickButton(panel, 0);

    /* Subsystems */
    private final Swerve s_Swerve = new Swerve();
    private final pVision vision = new pVision();
    private final Grabber claw = new Grabber();
    private final ArmControl arm = new ArmControl();

    /* Troubleshooting, Auto, and Shuffleboard */
    private final SendableChooser<Command> commandChooser = new SendableChooser<>();
    private GenericEntry autoCommand;
    private GenericEntry autoDelay;
    private GenericEntry autoHold;

    PathPlannerTrajectory CenterRB = PathPlanner.loadPath("CENTERRB", new PathConstraints(2, 2));

    PathPlannerTrajectory CenterMO = PathPlanner.loadPath("CENTERMO", new PathConstraints(2,2));
    
    PathPlannerTrajectory CenterPSTraj = PathPlanner.loadPath("CENTERPS", new PathConstraints(2, 2));
    
    PathPlannerTrajectory CenterS = PathPlanner.loadPath("CENTERS", new PathConstraints(2, 2));
    
    PathPlannerTrajectory CenterB = PathPlanner.loadPath("CENTERSB", new PathConstraints(2, 2));
    
    PathPlannerTrajectory CenterSPTraj = PathPlanner.loadPath("CENTERSP", new PathConstraints(2, 2));

    PathPlannerTrajectory LeaveC = PathPlanner.loadPath("LEAVEC", new PathConstraints(2, 2));
    
    PathPlannerTrajectory LeftBPPTraj = PathPlanner.loadPath("LEFTB", new PathConstraints(2, 2));
    
    PathPlannerTrajectory LeftP = PathPlanner.loadPath("LEFTP", new PathConstraints(2, 2));
    
    PathPlannerTrajectory LeftPS = PathPlanner.loadPath("LEFTPS", new PathConstraints(2, 2));
    
    PathPlannerTrajectory LeftSPPTraj = PathPlanner.loadPath("LEFTS", new PathConstraints(2, 2));

    PathPlannerTrajectory ReturnC = PathPlanner.loadPath("RETURNC", new PathConstraints(2, 2));
    
    PathPlannerTrajectory RightB = PathPlanner.loadPath("RIGHTB", new PathConstraints(2,2));
    
    PathPlannerTrajectory RightP = PathPlanner.loadPath("RIGHTP", new PathConstraints(2, 2));
    
    PathPlannerTrajectory RightPS = PathPlanner.loadPath("RIGHTPS", new PathConstraints(2, 2));
    
    PathPlannerTrajectory RightS = PathPlanner.loadPath("RIGHTS", new PathConstraints(2, 2));

    //PathPlannerTrajectory CenterB = PathPlanner.loadPath("CENTERB", new PathConstraints(2, 2));

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
                () -> -driver.getRawAxis(rotationAxis), 
                () -> robotCentric.getAsBoolean(),
                () -> goSpeed.getAsBoolean()                
            )
        );

        // arm will look at holding the position.
        arm.setDefaultCommand(new defaultArm(arm, board));
        
        // No default for gripper - it will run a command until it is done
        // or interupted.

        ///// COMMAND BUTTONS


        // TODO: ORGANIZE THE BUTTONS!!!!!!

        // Set Driver Commands
        doDriverCompetitionCommands();

        doDriverTestCommands();



        // ARM commands

        doArmCompetitionCommands();

        doArmTestCommands();



        // Gripper Test Commands

        doGripperCompetitionCommands();

        doGripperTestCommands();


        // Configure the button bindings
        configureButtonBindings();

        // run shuffleboard essentials
        runShuffleboardGetInfo();

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
        int pov = driver.getPOV(0);

        if (pov == 0){
            System.out.println("test0");
        }

        else if (pov == 180){
            System.out.println("test180");
        }

        /* Driver Buttons */

        counterAccel.whileTrue(new balanceAuto(s_Swerve).repeatedly().until(() -> s_Swerve.isRobotLevel()).andThen(new InstantCommand(() -> System.out.println("Balanced"))));

        slowForward.whileTrue(new gotoArmGeneralLocation(arm,50,90).repeatedly());
        

<<<<<<< HEAD
        /*while(arm.panelMove(panel.getRawAxis(panelY))){
=======
    }


    /////////////ARM

    private void doArmCompetitionCommands() {




    }

    private void doArmTestCommands() {


        // From Tim: See this. This is how to do this.
        // shoulderAdjUp.debounce(0.04).onTrue( new {Your Command here})
        // shoulderAdjDown.debounce(0.04).onTrue ( new {Your command here})

        while(arm.panelMove(panel.getRawAxis(panelY))){
>>>>>>> e3e1d35e21d0cfcc429fe8a8d439fdc4685db846
            if (arm.panelMoveDecision(panel.getRawAxis(panelY))) {
                arm.goToElbowSetpoint(arm.elbowCurrentAngle() + 5);
            }else {
                arm.goToElbowSetpoint(arm.elbowCurrentAngle() - 5);
                
            }
        }


        while(arm.panelMove(panel.getRawAxis(panelX))){
            if (arm.panelMoveDecision(panel.getRawAxis(panelX))) {
                arm.goToShoulderSetpoint(arm.shoulderCurrentAngle() + 5);
            }else {
                arm.goToShoulderSetpoint(arm.shoulderCurrentAngle() - 5);
            }
        }*/
        
        


    }

    ///// GRABBER

    private void doGripperCompetitionCommands(){

    }

    private void doGripperTestCommands(){

       // CLAW COMMANDS
       clawOpen.debounce(0.04).whileTrue(new openClaw(claw).withTimeout(Constants.kGrabber.openTimeout));
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

        LeftS.debounce(0.04).whileTrue(new doPathTrajectory(s_Swerve,LeftSPPTraj).andThen(new doPathTrajectory(s_Swerve, LeftBPPTraj))); // Do the path plan

        CenterSP.debounce(0.04).whileTrue(new doPathTrajectory(s_Swerve, CenterSPTraj).andThen(new doPathTrajectory(s_Swerve, CenterPSTraj)));

        //CenterSB.debounce(0.04).whileTrue(new doPathTrajectory(s_Swerve, CenterB));

        //RightSPS.debounce(0.04).whileTrue(new doPathTrajectory(s_Swerve, RightS).andThen(new doPathTrajectory(s_Swerve, RightB)));

        CenterSB.debounce(0.04).whileTrue(new doPathTrajectory(s_Swerve, CenterS).andThen(new doPathTrajectory(s_Swerve, CenterB)));

        LeftSPS.debounce(0.04).whileTrue(new doPathTrajectory(s_Swerve, LeftSPPTraj).andThen(new doPathTrajectory(s_Swerve, LeftP).andThen(new doPathTrajectory(s_Swerve, LeftPS))));



        
        
    }
    private void runTroubleshooting() {

        Shuffleboard.selectTab("MAIN");
        SmartDashboard.putData(s_Swerve);
        SmartDashboard.putData(vision);

        // // This section will send commands to the shuffleboard. 
        // We will probably need to disable the bandwidth limitations.

        // commandChooser.setDefaultOption("Foo", new fooCommand());
        // commandChooser.addOption("Bar", new BarCommand());
        commandChooser.addOption("Lock Wheels", new swerveLockPosition(s_Swerve, 0.0));

        SmartDashboard.putData(CommandScheduler.getInstance());
        
    }


    private void runShuffleboardGetInfo() {
        
        autoCommand = Shuffleboard.getTab("MAIN")
        .add("AUTON COMMAND", "ALPHA")
        .withWidget(BuiltInWidgets.kComboBoxChooser)
        .getEntry();

        autoDelay = Shuffleboard.getTab("MAIN")
        .add("AUTON DELAY", 0)
        .withWidget(BuiltInWidgets.kNumberSlider)
        .withProperties(Map.of("min", 0, "max", 3))
        .getEntry();
      
        autoHold = Shuffleboard.getTab("MAIN")
        .add("AUTON LOCK WHEELS", false)
        .withWidget(BuiltInWidgets.kToggleSwitch)
        .getEntry();
      
                

  
    }

// attempt to read alliance color and number
public enum Alliance {
    Red1,
    Blue1,
    Red2,
    Blue2,
    Red3,
    Blue3,
    Invalid,
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

        if (test.getAsBoolean() == true){
           return new doPathTrajectory(s_Swerve, CenterS);
        }

        else{
            return (new doPathTrajectory(s_Swerve, CenterB));
        }

    }

    //            return new doPathTrajectory(s_Swerve, CenterS).andThen(new doPathTrajectory(s_Swerve, CenterB));


}
