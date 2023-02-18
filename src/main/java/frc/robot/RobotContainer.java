package frc.robot;

import java.util.Map;

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

    /* Drive Controls */
    private final int translationAxis = XboxController.Axis.kLeftY.value;
    private final int strafeAxis = XboxController.Axis.kLeftX.value;
    private final int rotationAxis = XboxController.Axis.kRightX.value;

    /* Driver Buttons */
    private final JoystickButton zeroGyro = new JoystickButton(driver, XboxController.Button.kY.value);
    private final JoystickButton robotCentric = new JoystickButton(driver, XboxController.Button.kLeftBumper.value);
    private final JoystickButton goSpeed = new JoystickButton(driver, XboxController.Button.kStart.value);
    private final JoystickButton counterAccel = new JoystickButton(driver, XboxController.Button.kBack.value);
    private final JoystickButton holdBot = new JoystickButton(driver, XboxController.Button.kA.value);
    private final JoystickButton goToTag = new JoystickButton(driver, XboxController.Button.kX.value);
    // Control Board Stuff//
    private final JoystickButton forwardShoulder = new JoystickButton(board, 2);
    private final JoystickButton backwardShoulder = new JoystickButton(board, 1);
    private final JoystickButton forwardElbow = new JoystickButton(board, 7);
    private final JoystickButton backwardElbow = new JoystickButton(board, 3);
    private final JoystickButton coneGrab = new JoystickButton(panel, 1);
    private final JoystickButton invertSwitchButton1 = new JoystickButton (board, 4);
    private final JoystickButton invertSwitchButton2 = new JoystickButton (board, 8);
    // When we get a new switch change button number to what the switch is//
    private final JoystickButton squareGrab = new JoystickButton(panel, 1);

    // Stick buttons
    private final JoystickButton LeftSPS = new JoystickButton(driver, 9);
    private final JoystickButton LeftS = new JoystickButton(stick, 11);
    private final JoystickButton CenterSP = new JoystickButton(stick, 12);
    private final JoystickButton RightSB = new JoystickButton(stick, 10);
    //private final JoystickButton CenterSB = new JoystickButton(stick, 9);

    // Switch buttons
    // private final JoystickButton forwardShoulder = new JoystickButton(panel, 1);

    /* Subsystems */
    private final Swerve s_Swerve = new Swerve();
    private final pVision vision = new pVision();

    /* Troubleshooting, Auto, and Shuffleboard */
    private final SendableChooser<Command> commandChooser = new SendableChooser<>();
    private GenericEntry autoCommand;
    private GenericEntry autoDelay;
    private GenericEntry autoHold;
    // auton paths
    PathPlannerTrajectory LeftSPPTraj = PathPlanner.loadPath("LEFTS", new PathConstraints(2, 2));
    PathPlannerTrajectory LeftBPPTraj = PathPlanner.loadPath("LEFTB", new PathConstraints(2, 2));
    PathPlannerTrajectory CenterSPTraj = PathPlanner.loadPath("CENTERSP", new PathConstraints(2, 2));
    PathPlannerTrajectory CenterPSTraj = PathPlanner.loadPath("CENTERPS", new PathConstraints(2, 2));
    PathPlannerTrajectory RightS = PathPlanner.loadPath("RIGHTS", new PathConstraints(2, 2));
    PathPlannerTrajectory RightB = PathPlanner.loadPath("RIGHTB", new PathConstraints(2,2));
    PathPlannerTrajectory LeftP = PathPlanner.loadPath("LEFTP", new PathConstraints(2, 2));
    PathPlannerTrajectory LeftPS = PathPlanner.loadPath("LEFTPS", new PathConstraints(2, 2));

    //PathPlannerTrajectory CenterB = PathPlanner.loadPath("CENTERB", new PathConstraints(2, 2));


    /** The container for the robot. Contains subsystems, OI devices, and commands. */
    public RobotContainer() {

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


        // untested goto commands.
        


        // Configure the button bindings
        configureButtonBindings();

        // run shuffleboard essentials
        runShuffleboardGetInfo();

        // run the shuffleboard
        runTroubleshooting();
    }



    /**
     * Use this method to define your button->command mappings. Buttons can be created by
     * instantiating a {@link GenericHID} or one of its subclasses ({@link
     * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
     * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
     **/
    private void configureButtonBindings() {
        
        /* Driver Buttons */
        zeroGyro.onTrue(new InstantCommand(() -> s_Swerve.zeroGyro()));

        goToTag.debounce(0.04).whileTrue(new chaseTagV2(vision.camera, s_Swerve));

        // counterAccel.whileTrue(new balanceAuto(s_Swerve).repeatedly());
        counterAccel.whileTrue(new Balance(s_Swerve));

        holdBot.debounce(0.04).whileTrue(new swerveLockPosition(s_Swerve, rotationAxis));

        LeftS.debounce(0.04).whileTrue(new doPathTrajectory(s_Swerve,LeftSPPTraj).andThen(new doPathTrajectory(s_Swerve, LeftBPPTraj))); // Do the path plan

        CenterSP.debounce(0.04).whileTrue(new doPathTrajectory(s_Swerve, CenterSPTraj).andThen(new doPathTrajectory(s_Swerve, CenterPSTraj)));

        //CenterSB.debounce(0.04).whileTrue(new doPathTrajectory(s_Swerve, CenterB));

        RightSB.debounce(0.04).whileTrue(new doPathTrajectory(s_Swerve, RightS).andThen(new doPathTrajectory(s_Swerve, RightB)));

        LeftSPS.debounce(0.04).whileTrue(new doPathTrajectory(s_Swerve, LeftSPPTraj).andThen(new doPathTrajectory(s_Swerve, LeftP).andThen(new doPathTrajectory(s_Swerve, LeftPS))));

        //Control Board//

        forwardShoulder.debounce(0.04).whileTrue(new shoulderMove());

        backwardShoulder.debounce(0.04).whileTrue(new shoulderMoveV2());

        forwardElbow.debounce(0.04).whileTrue(new elbowMove());

        backwardElbow.debounce(0.04).whileTrue(new elbowMoveV2());

        coneGrab.debounce(0.04).whileTrue(new coneMove());

        coneGrab.debounce(0.04).whileFalse(new coneRelease());

        squareGrab.debounce(0.04).whileTrue(new squareMove());

        squareGrab.debounce(0.04).whileFalse(new squareRelease());

        invertSwitchButton1.debounce(0.04).whileTrue(new invertSwitch());

        invertSwitchButton2.debounce(0.04).whileTrue(new invertSwitchV2());

        
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




// // attempt to read alliance color and number
// public enum Alliance {
//     Red1,
//     Blue1,
//     Red2,
//     Blue2,
//     Red3,
//     Blue3,
//     Invalid,
// }

// public static Alliance getAlliance() {
             
//     AllianceStationID allianceID = DriverStationJNI.getAllianceStation();
    
//     //numbers 1-3 are for red and numbers 4-6 are for blue
//     //waiting for Tim
//     boolean isRedAlliance = false;
//     int stationNumber = 1;
//     switch (allianceID) {                 
//         case Red1:                     
//             isRedAlliance = true;
//             stationNumber = 1;
//             break;
//        case Red2:                    
//            isRedAlliance = true;
//            stationNumber = 2;
//            break;
//         case Red3:                     
//             isRedAlliance = true;
//             stationNumber = 3;
//             break;
//         case Blue1:                     
//             isRedAlliance = false;
//             stationNumber = 4;
//             break;
//         case Blue2:                     
//             isRedAlliance = false;
//             stationNumber = 5;
//             break;
//         case Blue3:                     
//             isRedAlliance = false;
//             stationNumber = 6;
//             break;
//           default:
//             return Alliance.Invalid;
//         }
//       }




    

    public Command getAutonomousCommand() {

        // // First, find the info that we need to choose the command.

        // String thisCmd =  autoCommand.getString("ALPHA");
        // double thisDelay =     autoDelay.getDouble(0);
        // boolean thisLock =  autoHold.getBoolean(false);

        // // Initalize the command
        // Command cmd = new WaitCommand(thisDelay);

        // // It may be worth making all the different traj's into their own command files.
        // // otherwise, look into using the .andThen / .parrallel / .raceParallel cmd compositions.

        // switch (thisCmd.charAt(0)) {
        //     case 'A':
        //         // default trajectory
        //         cmd=cmd.andThen(new backAndForth(s_Swerve));

        //     case 'B':
        //         // secondary traj

        //        cmd=cmd.andThen(new doTrajectory(s_Swerve, traj.shuffleLeft));

        //     case 'C':
        //         // 3rd traj
        //         cmd=cmd.andThen(new doTrajectory(s_Swerve, traj.shuffleRight));

        //     case 'D':
        //         // 4th traj

        //         cmd=cmd.andThen(new doTrajectory(s_Swerve, traj.exampleTrajectory));

        //     break;

        //     default:

        //         // Always have a backup plan. Don't rely on the shuffleboard.

        //         new backAndForth(s_Swerve);

        //     break;
        // }

        // // and keep adding commands
        // // cmd.andThen(null)

        // // In this case, we need to do testning to see if we should lock the wheels or continue to be in "balance" mode.
        // // Would reccommend for auto balance, then lock. Do not move after. 
        // // Another bot can ram into the side after we are already up and you will end up on top of it.
        // // if another bot wants to get up there, it can get up and try to push us.



        // if (thisLock) {
        //     cmd=cmd.andThen(new swerveLockPosition(s_Swerve,0));
        // }
                        
        
            // if(team=1){
            //     LeftS.debounce(0.04).whileTrue(new doPathTrajectory(s_Swerve,LeftSPPTraj).andThen(new doPathTrajectory(s_Swerve, LeftBPPTraj))); // Do the path plan
            // }
        

        return new doPathTrajectory(s_Swerve, LeftSPPTraj).andThen(new doPathTrajectory(s_Swerve, LeftP).andThen(new doPathTrajectory(s_Swerve, LeftPS)));

        

    }


}
