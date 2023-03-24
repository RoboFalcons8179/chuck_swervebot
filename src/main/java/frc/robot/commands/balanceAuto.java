// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.interfaces.Gyro;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.subsystems.Swerve;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class balanceAuto extends SequentialCommandGroup {
  /** Creates a new balanceAuto. */
  public balanceAuto(Swerve s_Swerve) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    if (!s_Swerve.isRobotLevel()) {
      addCommands(

        new InstantCommand(() -> System.out.println("------------------")),

        // new InstantCommand(() -> System.out.println(s_Swerve.isRobotLevel())),
          
//        new Balance(s_Swerve).withTimeout(0.425),
        new Balance(s_Swerve),

        new InstantCommand(() -> s_Swerve.stop())
        
        // new WaitCommand(1.0),

        // new WaitCommand(10).until(() -> (s_Swerve.isRobotStill()))
            
      );
    } else { // check if the robot is level when running the command
      addCommands(
        new InstantCommand(() -> System.out.println("did not run, robot is balanced")),

        new WaitCommand(1.00)
      );
    }
      

      // From Tim: consider an add in a turn the wheels apart (or 90 degrees) 
      // command to the Swerve system that would help prevent movement. 

      // Also look at Swerve.driveManual and Swerve.driveExtraManual
  }

}
