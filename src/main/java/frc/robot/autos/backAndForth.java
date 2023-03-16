// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.autos;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.*;
import frc.robot.commands.armStuff.*;
import frc.robot.subsystems.*;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.subsystems.Swerve;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class backAndForth extends SequentialCommandGroup {
  /** Creates a new backAndForth. */
  public backAndForth(Swerve s_Swerve, ArmControl arm, Grabber claw) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(

      new WaitCommand(1),

      new closeClaw(claw).withTimeout(0.5),
    
      //new doTrajectory(s_Swerve, traj.makeZ),

      new updateHoldPosition(() -> -6, () -> 45, arm),

      new WaitCommand(.5),

      new updateHoldPosition(() -> 115, () -> 45, arm),
      
      new WaitCommand(1),
      new updateHoldPosition(() -> 115, () -> 150, arm),

      new WaitCommand(1.5),

      new openClaw(claw).withTimeout(1),

      new updateHoldPosition(() -> 115, () -> 65, arm),

      new closeClaw(claw).withTimeout(1.5),

      
      
      //new WaitCommand(1.5),

      new updateHoldPosition(() -> -6, () -> 65, arm),

      new closeClaw(claw).withTimeout(1)
      
      );



    //addCommands( new doTrajectory(s_Swerve, traj.makeZFlipped));

  }
}

