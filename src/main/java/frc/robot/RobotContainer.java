package frc.robot;

import java.util.Map;

import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.event.NetworkBooleanEvent;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
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

    /* Drive Controls */
    private final int translationAxis = XboxController.Axis.kLeftY.value;
    private final int strafeAxis = XboxController.Axis.kLeftX.value;
    private final int rotationAxis = XboxController.Axis.kRightX.value;

    /* Driver Buttons */
    private final JoystickButton zeroGyro = new JoystickButton(driver, XboxController.Button.kY.value);
    private final JoystickButton robotCentric = new JoystickButton(driver, XboxController.Button.kLeftBumper.value);
    private final JoystickButton goSpeed = new JoystickButton(driver, XboxController.Button.kStart.value);
    private final JoystickButton counterAccel = new JoystickButton(driver, XboxController.Button.kBack.value);
    private final JoystickButton holdBot = new JoystickButton(driver, XboxController.Button.kRightBumper.value);
    private final JoystickButton goToTag = new JoystickButton(driver, XboxController.Button.kX.value);
    //private final JoystickButton LeftS = new JoystickButton(driver, XboxController.Button.kA.value);

    /* Subsystems */
    private final Swerve s_Swerve = new Swerve();
    private final pVision vision = new pVision();

    /* Troubleshooting, Auto, and Shuffleboard */
    private final SendableChooser<Command> commandChooser = new SendableChooser<>();
    private GenericEntry autoCommand;
    private GenericEntry autoDelay;
    private GenericEntry autoHold;
    


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
     */
    private void configureButtonBindings() {
        
        /* Driver Buttons */
        zeroGyro.onTrue(new InstantCommand(() -> s_Swerve.zeroGyro()));

        goToTag.whileTrue(new chaseTagV2(vision.camera, s_Swerve));

        counterAccel.whileTrue(new balanceAuto(s_Swerve).repeatedly());

        holdBot.whileTrue(new swerveLockPosition(s_Swerve, rotationAxis));

        //LeftS.whileTrue(new LEFTS(s_Swerve, rotationAxis));
    }

    // Runs a ton of smart dashboard commands. Lets you track status of commands.
    private void runTroubleshooting() {

        Shuffleboard.selectTab("MAIN");
        SmartDashboard.putData(s_Swerve);
        SmartDashboard.putData(vision);

        // // This section will send commands to the shuffleboard. 
        // We will probably need to disable the bandwidth limitations.

        // commandChooser.setDefaultOption("Foo", new fooCommand());
        // commandChooser.addOption("Bar", new BarCommand());
        commandChooser.addOption("Lock Wheels", new swerveLockPosition(s_Swerve, 0.0));


    }


    private void runShuffleboardGetInfo() {
        
        autoCommand = Shuffleboard.getTab("MAIN")
        .addPersistent("AUTON COMMAND", "ALPHA")
        .withWidget(BuiltInWidgets.kComboBoxChooser)
        .getEntry();

        autoDelay = Shuffleboard.getTab("MAIN")
        .addPersistent("AUTON DELAY", 0)
        .withWidget(BuiltInWidgets.kNumberSlider)
        .withProperties(Map.of("min", 0, "max", 3))
        .getEntry();
      
        autoHold = Shuffleboard.getTab("MAIN")
        .addPersistent("AUTON LOCK WHEELS", false)
        .withWidget(BuiltInWidgets.kToggleSwitch)
        .getEntry();
      
                

  
    }

      
    

    public Command getAutonomousCommand() {

        // First, find the info that we need to choose the command.

        String thisCmd =  autoCommand.getString("ALPHA");
        double thisDelay =     autoDelay.getDouble(0);
        boolean thisLock =  autoHold.getBoolean(false);

        // Initalize the command
        Command cmd = new WaitCommand(thisDelay);

        // It may be worth making all the different traj's into their own command files.
        // otherwise, look into using the .andThen / .parrallel / .raceParallel cmd compositions.

        switch (thisCmd.charAt(0)) {
            case 'A':
                // default trajectory
                cmd=cmd.andThen(new backAndForth(s_Swerve));

            case 'B':
                // secondary traj

               cmd=cmd.andThen(new doTrajectory(s_Swerve, traj.shuffleLeft));

            case 'C':
                // 3rd traj
                cmd=cmd.andThen(new doTrajectory(s_Swerve, traj.shuffleRight));

            case 'D':
                // 4th traj

                cmd=cmd.andThen(new doTrajectory(s_Swerve, traj.exampleTrajectory));

            break;

            default:

                // Always have a backup plan. Don't rely on the shuffleboard.

                new backAndForth(s_Swerve);

            break;
        }

        // and keep adding commands
        // cmd.andThen(null)

        // In this case, we need to do testning to see if we should lock the wheels or continue to be in "balance" mode.
        // Would reccommend for auto balance, then lock. Do not move after. 
        // Another bot can ram into the side after we are already up and you will end up on top of it.
        // if another bot wants to get up there, it can get up and try to push us.



        if (thisLock) {
            cmd=cmd.andThen(new swerveLockPosition(s_Swerve,0));
        }
                

        return cmd;

    }


}
